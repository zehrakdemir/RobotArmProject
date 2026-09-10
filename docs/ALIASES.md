movecobot projesi - olusturulan aliaslar
==========================================

movecobot
---------
Kol + Jodell gripper icin birlesik interaktif kontrol araci. Jog (eklem/
cartesian/TCP-yerel), hareket kaydetme (isim vererek -> otomatik alias),
kayitli hareket oynatma, manuel/drag-teach (elle hareket ettirme) modu.
On kosul: fairino_launch calisir durumda olmali.
Kullanim: movecobot
          movecobot --list            (kayitli hareketleri listeler)
          movecobot --replay <isim>   (kayitli bir hareketi dogrudan oynatir)

grippergui
----------
Jodell EPG40-050 gripper icin slider'li basit Tkinter arayuzu (pozisyon/
hiz/kuvvet). movecobot/movegripper ile AYNI ANDA, ayri bir surec olarak
CALISTIRMA - ayni robota iki ayri baglantidan komut gitmesi sorun cikarabilir.
Kullanim: grippergui

clearcobot
----------
movecobot'un ros2_control_node "respawn" mekanizmasindan (manuel moddan
cikista ROS2 kontrolu yeniden aktive edilemezse devreye giren yedek meknizma)
kalabilecek SAHIPSIZ surecleri temizler. Calisan fairino_launch/movecobot
oturumlarina dokunmaz, sadece bilgi verir.
Kullanim: clearcobot

clearfault
----------
fairino_launch CALISIRKEN (onu durdurmadan) robot fault verip WebApp'te
"Clear" hicbir sey yapmiyormus gibi gorunuyorsa kullan - genelde sebep,
ros2_control'un 100Hz'lik komut dongusunun hatayi temizler temizlemez
ayni/eski hedefe tekrar komut gonderip aninda geri getirmesi. Bu, o
donguyu GECICI durdurur, Clear'a basmani bekler (Enter'a basinca), sonra
geri acar. fairino_launch'i durdurmaz/yeniden baslatmaz - clearcobot'tan
farkli olarak (o SAHIPSIZ surecleri temizler), bu CALISAN ros2_control'u
gecici olarak durdurup acar.
Kullanim: clearfault

konumcobot
----------
Kolun (J1-J6 + TCP X/Y/Z/RX/RY/RZ) ve varsa gripper'in su anki konumunu
hizlica gosterip cikar. movecobot'un aksine ROS2/MoveIt/fairino_launch
GEREKTIRMEZ - dogrudan SDK uzerinden okur, bu yuzden aninda calisir. Sadece
"konum nedir" diye bakmak icin movecobot'un tum menusunu acmak istemedigin
durumlar icin.
Kullanim: konumcobot

Kayitli hareket aliaslari (otomatik)
-------------------------------------
movecobot'ta [4] Yeni hareket kaydet ile bir hareket dizisine isim verip
kaydettiginde, o isim OTOMATIK olarak bir alias haline gelir (~/.bashrc'de
"movecobot kayitli hareketler" blogu) - terminale sadece o ismi yazinca
(ornegin: deneme) hareket dogrudan, etkilesime girmeden oynatilir. Guncel
listeyi gormek icin: movecobot --list

Not: Yeni acilan terminallerde aliaslar otomatik hazir olur. Mevcut acik bir
terminalde hemen kullanmak icin: source ~/.bashrc
