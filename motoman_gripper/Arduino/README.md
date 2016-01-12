# Arduino
Arduinoに対してコマンドラインから
- コンパイル
- 書き込み
- シリアル通信

ができるようにするものである。

#####Environment
OS : Ubuntu 14.04 LTS

#####Links
[Emacs Arduino Support](http://www.emacswiki.org/emacs/ArduinoSupport)  
[github : Arduino-Makefile](https://github.com/sudar/Arduino-Makefile)


##Install
まずは、```arduino-mk```をインストールしましょう。
```
sudo apt-get install arduino-mk
```
また、シリアル通信には```screen```コマンドが必要です。  
まだインストールしていない場合は、
```
sudo apt-get install screen -y
```
でインストールしましょう。  
基本的に、```make monitor```すると、シリアル通信が始まります。  
終了したいときには、```Ctrl+a k```で終了するかどうか聞かれるので、```y```を入力後、```<RET>```で終了します。  
最後に、git cloneでダウンロードしてください。
```
git clone https://github.com/RyodoTanaka/Arduino/
```

##Usage
1. .inoファイルの編集
2. Makefileの編集
3. コンパイル
 - コンパイルのみ
 ```
 make
 ```
 - 書き込み
 ```
 make upload
 ```
 - シリアル通信
 ```
 make monitor
 ```
