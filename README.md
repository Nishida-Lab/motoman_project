# Motoman Project
Motoman(特に SIA5)のためのリポジトリです。
![SIA5](.image/sia5.png)

# Installation
0. `catkin_ws`作ってちょ
   やり方は、ROSの公式サイトとか見てね。

1. `clone`してちょ
   ```bash
   cd <catkin_ws>/src
   git clone https://github.com/RyodoTanaka/motoman.git
   ```

2. `wstool`してちょ
   ```bash
   cd <catkin_ws>
   wstool init src src/motoman_project/.rosinstall
   ```

3. `rosdep`してちょ
   ```bash
   cd <catkin_ws>
   rosdep install -i --from-paths src
   ```

4. `catkin_make`してちょ
   ```bash
   cd <catkin_ws>
   catkin_make
   ```
