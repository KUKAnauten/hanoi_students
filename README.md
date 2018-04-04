# Einleitung
Im Rahmen dieses Versuchs soll das Knobelspiel [Türme von Hanoi](https://de.wikipedia.org/wiki/T%C3%BCrme_von_Hanoi "Wikipedia: Türme von Hanoi") mit einem Roboterarm mit sieben Freiheitsgraden gelöst werden. Dazu muss nicht nur ein Algorithmus zur Lösung des theoretischen Problems implementiert werden, sondern auch die Schwierigkeiten gelöst werden, die bei der Bedienung eines Roboters bei einer solchen Aufgabenstellung entstehen.

Ziel des Rätsels ist, einen Stapel Scheiben von einem Turm zu einem anderen zu verschieben und bei jedem Zug stets nur eine Scheibe zu verschieben. Weiterhin dürfen Scheiben nur auf einen leeren Stab oder auf einem mit einer größeren Scheibe abgelegt werden.

# Umgebung
Als Roboter wird hier der _KUKA iiwa_ verwendet, der einen _Robotiq_ Dreifingergreifer montiert hat. Das Gesamtsystem ist in einer [ROS](http://www.ros.org/ "ROS-website")-Umgebung eingebunden. Dabei handelt es sich um ein Software-Framework, in dem einzelne Prozesse, sogenannte _Nodes_ bestimmte Aufgaben wie Hardwareabstraktion oder reine Datenverarbeitung übernehmen können und sich über Nachrichten austauschen können. Die einzelnen Prozesse/Aufgaben sind in Paketen organisiert. Hierbei werden [Robotiq](https://github.com/ros-industrial/robotiq)-Pakete zur Ansteuerung des Greifers über EtherCAT und Pakete des [iiwa_Stacks](https://github.com/IFL-CAMP/iiwa_stack) zur Ansteuerung des Roboters verwendet. Zur Bahnplanung des Roboters wird das [MoveIt! Motion Planning Framework](https://moveit.ros.org/) verwendet. Um die Einarbeitungszeit zu verkürzen und verschiedene Ansteuerungsarten des Roboters vereinheitlicht zu verwenden, wurden die benötigten Funktionen von uns noch weiter abstrahiert und stehen in der Klasse _RobotInterface_ im Paket [iimoveit](https://github.com/KUKAnauten/iimoveit) zur Verfügung. Eine Verknüpfung zur Dokumentation der API befindet sich im Paketordner von __hanoi\_students__ (siehe übernächster Absatz) oder ist [hier zu erreichen](https://htmlpreview.github.io/?https://raw.githubusercontent.com/KUKAnauten/iimoveit/master/doc/html/c%2B%2B/classiimoveit_1_1RobotInterface.html "RobotInterface API").

In ROS arbeitet man in sogenannten _Workspaces_. Für diesen Versuch wurde bereits ein Workspace eingerichtet, in dem die oben beschriebenen Pakete bereits enthalten sind. Auch wurde ein Paket erstellt, in dem sich der _Node_ befindet, der den Roboter und den Greifer steuern soll. Der Code dieses Nodes muss von Euch vervollständigt werden, sodass der Roboter die Türme von Hanoi erfolgreich löst.

Euer Workspace heißt __hanoi\_ws__ und befindet sich unter __/home/mlab8/ROS/__. Ihr werdet nur die Datei __hanoi\_iiwa\_node.cpp__ bearbeiten, die sich im Ordner __src__ des Pakets __hanoi\_students__ befindet. Diese kann mit einem Texteditor eurer Wahl geöffnet werden. Vorinstalliert ist Sublime Text, der auch Syntax Highlighting beherrscht.

# Schreiben des Programms
## Programmstruktur
Zur Durchführung der Aufgabe wurde die Klasse `HanoiRobot` erstellt, die alle Methoden von [RobotInterface](https://htmlpreview.github.io/?https://raw.githubusercontent.com/KUKAnauten/iimoveit/master/doc/html/c%2B%2B/classiimoveit_1_1RobotInterface.html "RobotInterface API") erbt.

___Achtung!___ Methoden, die mit _publish_ oder _run_ beginnen, sollen im Rahmen dieses Versuchs jedoch ignoriert werden, denn hierbei wird der Bahnplanungsschritt umgangen und kann zu unkontrollierbarem Verhalten führen! MoveIt! kann zwar Trajektorien berechnen, die Kollisionen vermeiden, jedoch müssen dazu Objekte über Sensoren erkannt werden oder per Hand in die Planungsumgebung eingefügt werden. Dies ist in unserer Umgebung noch nicht der Fall, weshalb geplante Bewegungen vor der Ausführung bestätigt werden müssen und auch zuerst im Simulator Gazebo geprüft werden sollen.
Zum Bewegen sollten momentan am besten nur die Methoden `planAndMove`, `planAndMoveToBasePose` sowie `moveAlongCartesianPathInWorldCoords` verwendet werden. Diese sind zur durchführung des Versuchs ausreichend und andere Methoden befinden sich noch in experimentellem Status.

# Ausführen des Codes
## Starten der ROS-Umgebung und benögtiter Nodes
Um das geschriebene Programm auszuführen, müssen zuerst die benötigten Nodes gestartet werden. Dazu geht man wie folgt vor:

1. Falls mit dem echten Roboter gearbeitet werden soll, muss zuerst der Roboter hochgefahren werden
2. Im Dateibrowser zum Workspace wechseln, Rechtsklick in einen leeren Bereich -> Terminal hier öffnen
3. Als erstes wird per `roscore` der Hauptprozess von ROS gestartet
4. Mit \[STRG\]+\[SHIFT\]+\[TAB\] einen neuen Tab öffnen, dann über den Befehl `sudoros` Adminrechte für dieses Terminal erlangen und mit diesen dann per `roslaunch robotiq_s_model_control s_model_ethercat.launch` den Node zur Steuerung des Greifers starten. Hier kann es gut sein, dass die EtherCAT Verbindung nicht aufgebaut wird. Dann muss man so lange den Befehl ausführen (mit der Pfeil-nach-oben-Taste den letzten Befehl in den Eingabebereich holen und mit Return bestätigen), bis die Meldung kommt, dass ein Slave gefunden und konfiguriert wurde.
4. Neuen Tab öffnen, per `roslaunch iiwa14_s_model_moveit run_move_group.launch` alle nötigen Nodes und Konfigurierungen starten, um den Roboter anzusteuern. Fall mit dem echten Roboter gearbeitet werden soll, muss an den Befehl ein `sim:=false` angehängt werden!
6. Falls im vorherigen Schritt `sim:=false` gesetzt wurde, muss jetzt die RobotApplication "ROSSmartServo" über das SmartPad des Roboters gestartet werden. Ansonsten wurde die Simulationssoftware Gazebo gestartet, in der ihr sehen könnt, wie sich der Roboter vorraussichtlich bewegen wird.
7. Als Nächstes könnt ihr nun euren Node starten, siehe nächster Abschnitt

## Kompilieren und Starten des Hanoi Nodes
Habt ihr euren Code geändert, müsst ihr ihn erst kompilieren (sowie assemblen und linken). Dazu öffnet ihr wieder einen neuen Tab und führt darin den Befehl `catkin build hanoi_students` aus. Enthält euer Code Fehler, wird der Compiler dies anzeigen und ihr müsst diese erst beheben.

Ist der Code (syntaktisch) fehlerfrei, könnt ihr ihn mit dem Befehl `roslaunch hanoi_students hanoi.launch` ausführen.

Im Fenster der Visualisierungssoftware RViz seht ihr dann die geplanten Bewegungen. Wenn für einen Befehl der Wert `approvalRequired` auf `true` gesetzt wurde, wird die Bewegung jedoch nicht ausgeführt. Ist man sich sicher, dass die Bewegung Kollisionsfrei durchgeführt werden kann, klickt man dann in RViz auf _Next_.
