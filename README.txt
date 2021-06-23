# Final-Project

這個Final Project裏面有分成五個部分：
1.跟著曲綫前進（line detection）
2.遇到障礙物，繞過障礙物（PING)
3.掃到April tag，去april tag隔壁進行定位
4.定位后再去到終點。
5.在開始和結束會用Zigbee回傳訊息給PC, 當上面每個部分結束是也回用Zigbee傳訊息給PC。

以上的1、3、4是透過OpenMV 也就是main.py進行。
2和4的部分用mbed 也就是main.cpp進行。

在main.cpp裏有宣告所有的rpc function,主要是被呼叫用來寫訊息到Zigbee。
還有PING detection的function和avoid_blocking(),當車子偵測到障礙物距離少於25cm就會進入到avoid_blocking(),這個funtion裏面所寫的是繞障礙物一圈。

OpenMV的main.py是寫了1、3和4部分，開始時車子用綫路設定初始值，當車子前進后隨著曲綫變化的差值與初始值比較，然後就能計算出車子應該偏轉的角度，使得車子隨著曲綫前進。
當1結束后就開始3，車子繞過和障礙物后就會掃到April tag，然後我用三角函數來推算前進的距離和偏轉的角度，以使得車子可以往April tag方向前進和定位，定位結束后再走向終點。
