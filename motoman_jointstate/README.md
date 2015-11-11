# Motoman JointState
Motoman(特に SIA5)のJointStateに関するテストリポジトリ

![](.image/motoman_JointState_test.gif)

# 各関節軸の角度を適当にPublish

```bash
rosrun motoman_jointstate joint_publisher.py
```

# 各関節の角度をSubscribe

```bash
rosrun motoman_jointstate joint_listener.py
```

# 全部Launch

```bash
roslaunch motoman_jointstate sia5_joint_demo.launch
```

# 軸の名前
xacroとかで出てくる軸名は安川独自のものです．
教えてくれた覚え方を書いておきます．

|軸名|フォルダ名|
|:--:|:--:|
|s|swing|
|l|lower|
|e|elbow|
|u|upper|
|r|rotate|
|b|bend|
|t|twist|
