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
各ラウンドでフィルタを初期化し直す。中央値、p95、プロセスCPU時間、
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
