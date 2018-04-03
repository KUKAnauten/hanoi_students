# Einleitung
Im Rahmen dieses Versuchs soll das Knobelspiel [Türme von Hanoi](https://de.wikipedia.org/wiki/T%C3%BCrme_von_Hanoi "Wikipedia: Türme von Hanoi") mit einem Roboterarm mit sieben Freiheitsgraden gelöst werden. Dazu muss nicht nur ein Algorithmus zur Lösung des theoretischen Problems implementiert werden, sondern auch die Schwierigkeiten gelöst werden, die bei der Bedienung eines Roboters bei einer solchen Aufgabenstellung entstehen.

Ziel des Rätsels ist, einen Stapel Scheiben von einem Turm zu einem anderen zu verschieben und bei jedem Zug stets nur eine Scheibe zu verschieben. Weiterhin dürfen Scheiben nur auf einen leeren Stab oder auf einem mit einer größeren Scheibe abgelegt werden.

# Umgebung
Als Roboter wird hier der _KUKA iiwa_ verwendet, der einen _Robotiq_ Dreifingergreifer montiert hat. Das Gesamtsystem ist in einer [ROS](http://www.ros.org/ "ROS-website")-Umgebung eingebunden. Dabei handelt es sich um ein Software-Framework, in dem einzelne Prozesse, sogenannte _Nodes_ bestimmte Aufgaben wie Hardwareabstraktion oder reine Datenverarbeitung übernehmen können und sich über Nachrichten austauschen können. Die einzelnen Prozesse/Aufgaben sind in Paketen organisiert. Hierbei werden [Robotiq](https://github.com/ros-industrial/robotiq)-Pakete zur Ansteuerung des Greifers über EtherCAT und Pakete des [iiwa_Stacks](https://github.com/IFL-CAMP/iiwa_stack) zur Ansteuerung des Roboters verwendet. Zur Bahnplanung des Roboters wird das [MoveIt! Motion Planning Framework](https://moveit.ros.org/) verwendet. Um die Einarbeitungszeit zu verkürzen und verschiedene Ansteuerungsarten des Roboters vereinheitlicht zu verwenden, wurden die benötigten Funktionen von uns noch weiter abstrahiert und stehen in der Klasse _RobotInterface_ im Paket [iimoveit](https://github.com/KUKAnauten/iimoveit) zur Verfügung. **TODO Link zur Doku, Codegerüst vorgegeben**

In ROS arbeitet man in sogenannten _Workspaces_. Für diesen Versuch wurde bereits ein Workspace eingerichtet, in dem die oben beschriebenen Pakete bereits enthalten sind. Auch wurde ein Paket erstellt, in dem sich der _Node_ befindet, der den Roboter und den Greifer steuern soll. Der Code dieses Nodes muss von Euch vervollständigt werden, sodass der Roboter die Türme von Hanoi erfolgreich löst.

Euer Workspace heißt __hanoi\_ws__ und befindet sich unter __/home/mlab8/ROS/__. Ihr werdet nur die Datei __hanoi\_iiwa\_node.cpp__ bearbeiten, die sich im Ordner __src__ des Pakets __hanoi\_students__ befindet. Diese kann mit einem Texteditor eurer Wahl geöffnet werden. Vorinstalliert ist Sublime Text, der auch Syntax Highlighting beherrscht.

# Ausführen des Codes
## Starten der ROS-Umgebung und benögtiter Nodes
Um das geschriebene Programm auszuführen, müssen zuerst die \[STRG\]

1. Roboter hochgefahren
2. Im Dateibrowser zum ws -> Rechtsklick, Terminal hier öffnen
3. roscore
4. neuer tab, sudoros, dann roslaunch robotiq_s_model_control s_model_ethercat.launch
5. neuer tab, roslaunch iiwa14_s_model_moveit run_move_group.launch [sim:=false]
6. Falls sim:=false -> RobotApplication auf SmartPad starten
7. neuer tab, roslaunch hanoi_students hanoi.launch