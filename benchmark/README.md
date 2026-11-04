# 2コアのコア処理ベンチマーク

ROS 2 Humble、Eigen、PCL、OpenMPの依存を導入したLinuxで実行する。
ベンチマークは通常ビルドでは無効。

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B .bench/tune -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_BENCHMARKS=ON -DBUILD_TESTING=ON
cmake --build .bench/tune -j 4
taskset -c 0,2 .bench/tune/benchmark_core /absolute/path/map.pcd
cd .bench/tune
ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 ctest --output-on-failure
```

`taskset`のCPU番号は実行ホストのトポロジーに合わせ、別々の物理コアを選ぶ。
ベンチマーク内部は`num_threads=2`、動的スレッド数調整なし。
本調査のDockerではLinuxの`lscpu`がCPU 0と2を別COREとして報告した。
仮想環境内のCPU制約であり、Windowsホストの専有物理コアを保証するものではない。

地図は0.2 mでボクセル化する。元地図の最初の有限点を原点とし、
70 m以内の有限点から等間隔に50点と2000点を選ぶ。
50粒子、固定seed、初期共分散0.01、移動量ゼロで、
各条件について5回のウォームアップ後に30更新を測り、5ラウンド実行する。
各ラウンドでフィルタを初期化し直す。中央値、p95、最大値、プロセスCPU時間、
推定値のchecksum、リセット回数を出す。
別途、50粒子分の固定問い合わせによる地図範囲判定と距離探索も測る。
この区間計測は更新処理と異なる固定問い合わせであり、区間時間の合計を
更新時間として扱わない。

改善前後のバイナリを別ビルドディレクトリに保存し、同じCPU・地図で交互に実行する。
地図ロードと入力生成は時間計測から除く。起動時間、DDS、TF、点群前処理、
実走行の精度・遅延はこのベンチマークの対象外。
checksumは比較の補助であり、精度の証明ではない。

本調査で使用した`map_tsudanuma.pcd`はワークスペース直下の外部データで、
このリポジトリには同梱しない。再実行時は同じPCDを別途用意して絶対パスを渡す。

## GPUとの比較

Vulkanの開発依存はCPU専用ビルドに含めない。GPUを含める場合だけ導入し、
別のビルドディレクトリを使う。

```bash
sudo apt-get install libvulkan-dev glslang-tools
cmake -S . -B .bench/vulkan -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_BENCHMARKS=ON -DBUILD_TESTING=ON -DEMCL3D_ENABLE_VULKAN=ON
cmake --build .bench/vulkan -j 4
taskset -c 0,2 .bench/vulkan/benchmark_core /absolute/path/map.pcd cpu
taskset -c 0,2 .bench/vulkan/benchmark_core /absolute/path/map.pcd vulkan
```

末尾の選択を省略するとCPU。出力には選択したバックエンドと実デバイス名を含む。
GPU更新時間には、問い合わせ準備、転送、GPU探索、同期・読み戻し、CPU側の
尤度計算・正規化・推定を含む。地図ロードと初回パイプライン生成は従来どおり除外する。
固定問い合わせの`phase=contains/distance`はスカラーCPU処理なので、
GPU実行を選択しても`execution=cpu`と明示する。

Vulkan有効時の`ctest`には実GPUを使う距離・フィルタ更新・丸め境界のテストが加わる。
ソフトウェアVulkanをGPU性能として扱わない。ドライバ、GPU名、CPU affinity、
同じ地図・点数・粒子数を記録し、CPUとGPUを同じ環境で比較する。

CPU専用の依存分離は、Vulkan検索を禁止したビルドでも確認できる。

```bash
cmake -S . -B .bench/cpu -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DEMCL3D_ENABLE_VULKAN=OFF -DCMAKE_DISABLE_FIND_PACKAGE_Vulkan=TRUE \
  -DBUILD_TESTING=ON -DBUILD_BENCHMARKS=ON
```
