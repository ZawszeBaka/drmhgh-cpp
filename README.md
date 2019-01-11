Team1
ws://127.0.0.1:9090

v2.0

roslaunch team200 team200.launch

Install OpenCV to virtualenv # recommended

pkg-config --modversion opencv // check opencv version
dpkg --get-selections | grep opencv // installed packages
dpkg -l | grep opencv // list all the packages
dpkg -L | grep opencv // list all installed packages


=====================================================================
Update Code :https://drive.google.com/drive/folders/1sqVxYwPfHmCDlFcriqGFqOyqtwIWCU62?usp=sharing- Retrieved Team1_image
==========

Anh gửi mẫu 1 package chuẩn nhé: chú ý tên đội không được viết hoa

CMakeLists.txt:
đổi project(team1) theo id đội vd project(team432)

package.xml: đổi <name>team1</name> thành <name>team432</name>
nên viêt lại các mục <description>....</description>
<maintainer email="....">...</maintainer>
team1.launch:đổi thành team432.launch, lấy 2 số cuối của id đội để đổi thành ns="bridge32", value="9032", <node name="team432" pkg="team432" type="team432_node"
