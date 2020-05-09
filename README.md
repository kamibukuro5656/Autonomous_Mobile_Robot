# 概要
ROSを使用して学習用に作成した自律移動型ロボットのソースコード。<br>
rviz上でゴールを与えると目標座標まで自律的に移動する。<br>
事前のマッピングが必要。<br>
学習用のため既存のROSノードは未使用。<br>

# 動画
 [![](http://img.youtube.com/vi/FAr_-Xh7VtE/0.jpg)](http://www.youtube.com/watch?v=FAr_-Xh7VtE "")

# 依存関係
- [jsk-ros-pkg/jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization)
- [ceres-solver](https://github.com/ceres-solver/ceres-solver)
- [nanoflann](https://github.com/jlblancoc/nanoflann)

# 使用法

# 参考文献
- 上田隆一, "詳解 確率ロボティクス Pythonによる基礎アルゴリズムの実装", 講談社, 2019年10月
- 友納正裕, "SLAM入門 -ロボットの自己位置推定と地図構築の技術-", オーム社, 2018年3月
- [Stefan Kohlbrecher, Oskar von Stryk, Johannes Meyer, Uwe Klingauf, "A Flexible and Scalable SLAM System with Full 3D Motion Estimation", IEEE International Symposium on Safety, Security, and Rescue Robotics, November 2011](https://www.researchgate.net/publication/228852006_A_flexible_and_scalable_SLAM_system_with_full_3D_motion_estimation)
- [Moritz Werling, Julius Ziegler, So¨ren Kammel, and Sebastian Thrun, "Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame", IEEE International Conference on Robotics and Automation, June 2010](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame)
- [Uk-Youl Huh, Seong-Ryong Chang, "A Collision-Free G² Continuous Path-Smoothing Algorithm Using Quadratic Polynomial Interpolation", International Journal of Advanced Robotic Systems 11(1):1, December 2014](https://www.researchgate.net/publication/269785358_A_Collision-Free_G_Continuous_Path-Smoothing_Algorithm_Using_Quadratic_Polynomial_Interpolation)
- [株丹亮, 西田健, "ポテンシャル関数を用いるT-RRTによる経路探索", 第33回日本ロボット学会学術講演会, 2015年9月](http://lab.cntl.kyutech.ac.jp/~nishida/paper/2015/3L3-03.pdf)
- [Atsushi Sakai, "MyEnigma"](https://myenigma.hatenablog.com/)