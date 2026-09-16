# Migrationsplan: LodeStar Odometry von ROS 1 Noetic (Ubuntu 20.04) nach ROS 2 Jazzy (Ubuntu 24.04)

Repository: `Lodestar-ros2-gnss-denied-pohang-test` (Paket `lodestar_odometry`)
Stand der Analyse: 2026-09-16, Basis = aktueller `main`

## Status: umgesetzt (2026-09-16)

Dieser Plan ist **vollständig implementiert**; das Repository befindet sich im beschriebenen Soll-Zustand. Was beim Umsetzen vom Plan abwich oder zusätzlich auffiel:

**Zusätzlich eingeführt.** `lodestar_odom::PrivateTopic()` in `utils.h` kapselt die `~/`-Regel für Topic-Namen, statt sie an 11 Stellen von Hand zu schreiben. Ein Unit-Check bestätigt, dass alle relativen Namen auf `/lodestar_odom_node/...` und alle absoluten Namen unverändert abbilden — die RViz-Konfiguration passt damit ohne Anpassung. `statistics.h` bekam eine zweite `ToMs()`-Überladung plus `ToMsSince()` für `std::chrono`, damit keine Zeitmessung versehentlich ROS- und Steady-Clock mischt.

**Beim Kompilieren gefundene Fehler im Plan.** `statistics.h` verwendet `std::cout/cerr/endl`, hatte aber nie ein eigenes `<iostream>` (unter ROS 1 kam es über `ros/ros.h`); `eval_trajectory.h` braucht `utils.h` für `PrivateTopic()`. Beides hätte den ersten `colcon build` abgebrochen. Ebenso ersetzt: das in PCL 1.15 entfallende `pcl/io/io.h` durch `pcl/common/io.h`.

**Bewusst nicht repariert.** In `processFrame` verdeckt eine innere Deklaration `bool success` die äußere, sodass das Registrierungsergebnis verworfen wird und `success` immer `true` bleibt. Das ist unverändert aus ROS 1 übernommen: Wer den Shadow entfernt, aktiviert den `exit(0)`-Zweig bei Registrierungsfehlern und ändert das Fusionsverhalten. Im Code steht ein erklärender Kommentar; die Korrektur gehört in eine eigene, bewusste Änderung.

**Verhaltensunterschied C++14 → C++17.** Die Konsolenzeile `... << frame << ... << ++frame/tot` war unter C++14 unsequenziert (undefiniert) und ist unter C++17 links-nach-rechts sequenziert. Der ausgegebene Frame-Index kann sich daher zwischen altem und neuem Build unterscheiden. Die Stelle ist jetzt explizit ausgeschrieben. Betroffen ist nur die Ausgabe, nicht die Trajektorie.

**Korrigierte Kleinigkeit.** Das Debug-Bild wurde mit Encoding `"MONO8"` statt `"mono8"` publiziert; kanonisch ist Kleinschreibung, sonst weisen Subscriber das Bild zurück. Betrifft nur `/radar_imported`.

**Was verifiziert wurde und was nicht.** Verifiziert auf Ubuntu 24.04 gegen die echten Zielbibliotheken (PCL 1.14.0, OpenCV 4.6.0, Ceres 2.2.0, Eigen 3.4.0): alle neun Übersetzungseinheiten kompilieren und linken zu einem lauffähigen Binary; ein Durchlauf über eine synthetische Radarsequenz erzeugt 24 wohlgeformte KITTI-Posen mit orthonormalen Rotationen (Abweichung 1e-6), ohne NaN, bei identischem Ergebnis über zwei Läufe; Valgrind meldet null Fehler. Die rclcpp-, tf2-, cv_bridge-, pcl_conversions- und rosbag2-Ebene wurde dabei gegen API-treue Stub-Header gebaut, weil `packages.ros.org` in der Build-Umgebung nicht erreichbar war. Das prüft Signatur- und Konsistenzfehler über alle Dateien hinweg, **ersetzt aber kein echtes `colcon build`** gegen ROS 2 Jazzy: die tatsächlichen rclcpp-Header können Überladungen strenger auflösen. Schritt 8 dieses Plans ist deshalb weiterhin auf deinem 24.04-Rechner auszuführen, ebenso der `evo`-Vergleich gegen die ROS-1-Referenztrajektorie — den kann nur ein ROS-1-Build liefern.

---

> **Wichtiger Hinweis zur Zielplattform:** ROS 2 Jazzy Jalisco wird offiziell nur auf **Ubuntu 24.04 (Noble)** unterstützt, nicht auf 22.04. Dieser Plan zielt daher auf **Jazzy + Ubuntu 24.04**. Wer zwingend auf 22.04 bleiben muss, nimmt entweder Humble (gleicher Plan, einige Header heißen dort noch `.h` statt `.hpp`, siehe Abschnitt 11) oder baut in einem `ros:jazzy`-Docker-Container.

---

## Inhalt

1. Ziel und Rahmenbedingungen
2. Ist-Analyse des Repositories
3. Zielstruktur des Repositories (Soll-Zustand)
4. Schritt 0 – Systemvorbereitung
5. Schritt 1 – `package.xml`
6. Schritt 2 – `CMakeLists.txt`
7. Schritt 3 – Zentrale Architekturentscheidung: Node-Handling
8. Schritt 4 – Allgemeine API-Übersetzungstabelle
9. Schritt 5 – Datei-für-Datei-Anweisungen
10. Schritt 6 – Launch, RViz, Startskript
11. Schritt 7 – Bag-Dateien konvertieren (ROS 1 → ROS 2)
12. Schritt 8 – Build, Test, Verifikation
13. Bekannte Stolpersteine und Unterschiede Humble/Jazzy
14. Abschluss-Checkliste

---

## 1. Ziel und Rahmenbedingungen

**Ziel:** Das Paket `lodestar_odometry` (Offline-Radar-Odometrie: LodeStar-Deskriptor + CFEAR-Point-Normal-Matcher) soll unter ROS 2 Jazzy mit `colcon` bauen, ROS-2-Bags lesen, dieselben Topics (für RViz2) publizieren und dieselben KITTI-Trajektoriendateien erzeugen wie unter ROS 1.

**Nicht-Ziel:** Der Algorithmus selbst (Polar-Transformation, Kreuzkorrelation, Contour/K-Nearest, Ceres-Registrierung) wird **nicht** verändert. Alles, was nur Eigen/PCL/OpenCV/Ceres nutzt, bleibt 1:1 erhalten.

**Bibliotheksversionen unter Ubuntu 24.04 / Jazzy (relevant für die Migration):**

| Bibliothek | Ubuntu 20.04 (alt) | Ubuntu 24.04 (neu) | Konsequenz |
|---|---|---|---|
| C++-Standard | C++14 (CMake-Flag) | **C++17 zwingend** (rclcpp, Ceres 2.2) | `CMAKE_CXX_STANDARD 17` |
| PCL | 1.10 | **1.14** | `Ptr` ist `std::shared_ptr` (seit 1.11) → `boost::shared_ptr` entfernen |
| OpenCV | 4.2 | **4.6** | `cv::linearPolar` ist deprecated, funktioniert aber noch |
| Ceres | 1.14 / 2.x | **2.2** | `ceres/manifold.h` vorhanden (wird bereits inkludiert); braucht C++17 |
| Eigen | 3.3 | 3.4 | keine Änderung |
| Boost | 1.71 | 1.83 | `program_options` bleibt, `boost::filesystem` → `std::filesystem` |
| Bag-Format | rosbag (.bag) | **rosbag2** (MCAP/SQLite3) | Bags müssen konvertiert werden |
| RViz | rviz | **rviz2** | `.rviz`-Datei konvertieren |

---

## 2. Ist-Analyse des Repositories

### 2.1 Struktur (aktuell)

```
Lodestar-ros2-gnss-denied-pohang-test/
├── CMakeLists.txt                     (catkin, cmake 2.8.3, C++14)
├── package.xml                        (format 2, catkin)
├── README.md
├── LICENSE
├── include/lodestar_odometry/
│   ├── eval_trajectory.h
│   ├── lodestar.h
│   ├── n_scan_normal.h
│   ├── odometrykeyframefuser.h
│   ├── pointnormal.h
│   ├── registration.h
│   ├── statistics.h
│   └── utils.h
├── src/
│   ├── lodestar_odom.cpp              (main: rosbag lesen, Pipeline treiben)
│   └── lodestar_odometry/
│       ├── eval_trajectory.cpp
│       ├── lodestar.cpp
│       ├── n_scan_normal.cpp
│       ├── odometrykeyframefuser.cpp
│       ├── pointnormal.cpp
│       ├── registration.cpp
│       ├── statistics.cpp
│       └── utils.cpp
├── launch/
│   ├── run_lodestar_odom              (Bash: roslaunch vis + rosrun lodestar_odom mit CLI-Args)
│   └── vis.launch                     (XML: startet rviz mit odom.rviz)
└── rviz/
    └── odom.rviz                      (ROS-1-RViz-Konfiguration)
```

### 2.2 ROS-1-Abhängigkeiten pro Datei (was migriert werden muss)

| Datei | ROS-1-Konstrukte | Aufwand |
|---|---|---|
| `statistics.h/.cpp` | `ros/ros.h`, `ros::Duration` in `ToMs()` | gering |
| `utils.h/.cpp` | `ros/ros.h`, `pcl_ros/point_cloud.h`, `pcl_ros/publisher.h` (nur Includes, kein Code) | gering |
| `pointnormal.h/.cpp` | `ros::NodeHandle nh("~")`, `static std::map<std::string, ros::Publisher> pubs`, `ros::Time::now()`, `ros::Duration(0)` bei Marker-Lifetime, `visualization_msgs::Marker/MarkerArray`, `geometry_msgs::Point`, `boost::shared_ptr` für `cell`/`MapPointNormal`, `pcl_conversions::fromPCL(..., ros::Time)` | mittel |
| `registration.h/.cpp` | `ros::NodeHandle nh_`, `ros::Publisher pub_association`, `visualization_msgs::MarkerArray`, `boost::shared_ptr<ceres::Problem>`, `boost::shared_ptr<Registration>` | gering |
| `n_scan_normal.h/.cpp` | nur Includes (`pcl_ros`, `ros/ros.h`), erbt von `Registration` | gering |
| `lodestar.h/.cpp` | `ros::NodeHandle`, `ros::Subscriber`, `ros::Publisher` (×5), `image_transport`, `cv_bridge`, `sensor_msgs::ImageConstPtr`, `nav_msgs::Odometry`, `tf::poseEigenToMsg`, `eigen_conversions`, `tf_conversions`, `tf/message_filter.h`, `tf/transform_listener.h`, `ros::Time::now()`, `pcl_conversions::toPCL(ros::Time,...)`, Publish von `pcl::PointCloud` direkt | hoch |
| `odometrykeyframefuser.h/.cpp` | `ros::NodeHandle`, Publisher/Subscriber, `tf::TransformBroadcaster`, `tf::transformEigenToTF`, `tf::StampedTransform`, `tf::poseEigenToMsg`, `laser_geometry`, `message_filters`, `std_msgs/Time.h`, `ros::TransportHints`, `boost::shared_ptr<n_scan_normal_reg>`, `ros::Time::now()`, `pcl::fromROSMsg` | hoch |
| `eval_trajectory.h/.cpp` | `ros::NodeHandle`, Publisher/Subscriber, `message_filters` (auskommentiert), `tf::TransformBroadcaster br`, `tf::poseMsgToEigen/poseEigenToMsg`, `boost::filesystem`, `ros::Time` in `poseStamped`-Typedef, `nav_msgs::Path` | mittel |
| `lodestar_odom.cpp` | `ros::init`, `ros::NodeHandle`, **`rosbag::Bag` / `rosbag::View`**, `BOOST_FOREACH`, `ros::ok()`, `ros::Time::now()`, `ros::Duration`, `eigen_conversions` | hoch |
| `CMakeLists.txt` | catkin, `catkin_package`, alte Komponentenliste | komplett neu |
| `package.xml` | format 2, catkin, ROS-1-Pakete (`roscpp`, `tf`, `tf_conversions`, `eigen_conversions`, `pcl_ros`, `graph_map` …) | komplett neu |
| `launch/vis.launch` | ROS-1-XML, `$(find …)`, `pkg="rviz"` | neu als `.launch.py` |
| `launch/run_lodestar_odom` | `roslaunch`, `rosrun`, ROS-1-Bag-Pfad | anpassen |
| `rviz/odom.rviz` | `rviz/PointCloud2`, `rviz/Odometry`, Topic-Format ROS 1 | konvertieren |

### 2.3 Nicht genutzte Abhängigkeiten (können ersatzlos entfallen)

- `laser_geometry` – nur inkludiert, nie verwendet.
- `image_transport` – `it`-Member und `pub` in `lodestar` werden nie benutzt.
- `message_filters` – in `eval_trajectory` nur auskommentiert.
- `rospy`, `genmsg`, `graph_map`, `std_msgs/Time.h`, `tf/message_filter.h`, `tf/transform_listener.h`.
- `pcl_ros` – wurde nur für `pcl_ros/point_cloud.h` (Publish von `pcl::PointCloud<T>` direkt) und `pcl_ros/transforms.h` gebraucht; unter ROS 2 wird stattdessen `pcl_conversions` + explizites `pcl::toROSMsg` verwendet.

### 2.4 Topics und Frames, die erhalten bleiben müssen (RViz-Kompatibilität)

Der Node heißt `lodestar_odom_node`. Unter ROS 1 wurden alle Publisher über `nh_("~")` (privater Namespace) erzeugt. Daraus ergeben sich diese effektiven Topic-Namen, die auch unter ROS 2 erhalten bleiben sollen:

| Publisher-Name im Code | ROS-1-effektiv | ROS-2-Name im Code | ROS-2-effektiv |
|---|---|---|---|
| `radar_odom` | `/lodestar_odom_node/radar_odom` | `"~/radar_odom"` | `/lodestar_odom_node/radar_odom` |
| `radar_odom_keyframe` | `/lodestar_odom_node/radar_odom_keyframe` | `"~/radar_odom_keyframe"` | gleich |
| `radar_registered` | `/lodestar_odom_node/radar_registered` | `"~/radar_registered"` | gleich |
| `radar_registered_keyframe` | `/lodestar_odom_node/radar_registered_keyframe` | `"~/radar_registered_keyframe"` | gleich |
| `path_est`, `map_cloud`, `associations` | `/lodestar_odom_node/…` | `"~/…"` | gleich |
| `/marine/Filtered`, `/radar_imported`, `/rot_lodestar`, `/current_normals` | absolut | unverändert (führender `/`) | gleich |

**Regel:** Topic-Strings ohne führenden `/` bekommen das Präfix `~/`; Strings mit führendem `/` bleiben unverändert. Frames: `world`, `sensor`, `sensor_est`, `radar_link` bleiben.

---

## 3. Zielstruktur des Repositories (Soll-Zustand)

```
Lodestar-ros2-gnss-denied-pohang-test/          ← wird als ~/ros2_ws/src/<repo> geklont
├── CMakeLists.txt                     ament_cmake, C++17
├── package.xml                        format 3, ament_cmake
├── README.md                          aktualisiert (colcon, ros2 run/launch, Bag-Konvertierung)
├── LICENSE
├── include/lodestar_odometry/
│   ├── eval_trajectory.h              (ROS-2-Header, rclcpp::Node::SharedPtr)
│   ├── lodestar.h
│   ├── n_scan_normal.h
│   ├── odometrykeyframefuser.h
│   ├── pointnormal.h
│   ├── registration.h
│   ├── statistics.h
│   └── utils.h
├── src/
│   ├── lodestar_odom.cpp              main: rclcpp::init, rosbag2_cpp::Reader
│   └── lodestar_odometry/*.cpp        (wie bisher, migriert)
├── launch/
│   ├── vis.launch.py                  startet rviz2 mit rviz/odom.rviz
│   └── lodestar_odom.launch.py        (optional) startet rviz2 + Node mit Parametern
├── scripts/
│   └── run_lodestar_odom.sh           ersetzt launch/run_lodestar_odom
├── rviz/
│   └── odom.rviz                      RViz2-Format
└── config/                            (optional)
    └── lodestar_params.yaml
```

Das Paket bleibt ein einziges Paket `lodestar_odometry` mit einer Bibliothek `lodestar_odometry` und einer Executable `lodestar_odom`. Der Ordnername des Repos ist für colcon irrelevant (maßgeblich ist `<name>` in `package.xml`).

---

## 4. Schritt 0 – Systemvorbereitung (Ubuntu 24.04)

```bash
# ROS 2 Jazzy (Desktop bringt rviz2, rosbag2, cv_bridge, pcl_conversions mit)
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools

# ROS-2-Pakete, die dieses Paket braucht
sudo apt install \
  ros-jazzy-rclcpp \
  ros-jazzy-rosbag2-cpp ros-jazzy-rosbag2-storage-mcap ros-jazzy-rosbag2-storage-default-plugins \
  ros-jazzy-sensor-msgs ros-jazzy-nav-msgs ros-jazzy-geometry-msgs ros-jazzy-std-msgs \
  ros-jazzy-visualization-msgs \
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-cv-bridge ros-jazzy-pcl-conversions ros-jazzy-angles \
  ros-jazzy-rviz2

# Systembibliotheken
sudo apt install libpcl-dev libopencv-dev libceres-dev libeigen3-dev \
  libboost-program-options-dev libomp-dev

# Für die Bag-Konvertierung (ROS1 .bag -> ROS2)
pip3 install --user rosbags      # oder: pipx install rosbags

# Workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repo-url> Lodestar-ros2-gnss-denied-pohang-test
source /opt/ros/jazzy/setup.bash
```

Prüfen:

```bash
pkg-config --modversion pcl_common   # 1.14.x
pkg-config --modversion opencv4      # 4.6.x
dpkg -s libceres-dev | grep Version  # 2.2.x
```

---

## 5. Schritt 1 – `package.xml` (komplett ersetzen)

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lodestar_odometry</name>
  <version>1.0.0</version>
  <description>Maritime radar odometry (LodeStar descriptor + CFEAR point-normal matcher), ROS 2 port</description>

  <maintainer email="dortz@snu.ac.kr">Hyesu Jang</maintainer>
  <maintainer email="michi.stober@gmail.com">Michael Stober</maintainer>
  <license>GPLv3</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rosbag2_cpp</depend>
  <depend>rosbag2_storage</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_eigen</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>pcl_conversions</depend>
  <depend>angles</depend>

  <depend>libpcl-all-dev</depend>
  <depend>libopencv-dev</depend>
  <depend>libceres-dev</depend>
  <depend>eigen</depend>
  <depend>libboost-program-options-dev</depend>

  <exec_depend>rviz2</exec_depend>
  <exec_depend>rosbag2_storage_mcap</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Entfernt gegenüber ROS 1: `catkin`, `roscpp`, `rospy`, `tf`, `tf_conversions`, `eigen_conversions`, `pcl_ros`, `laser_geometry`, `image_transport`, `graph_map`, `genmsg`.

---

## 6. Schritt 2 – `CMakeLists.txt` (komplett ersetzen)

```cmake
cmake_minimum_required(VERSION 3.16)
project(lodestar_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-sign-compare)

# ---- ROS 2 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(angles REQUIRED)

# ---- Systembibliotheken ----
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters kdtree search)
find_package(OpenCV 4 REQUIRED)
find_package(Ceres 2.0 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenMP REQUIRED)

add_definitions(${PCL_DEFINITIONS})

set(ROS_DEPS
  rclcpp rosbag2_cpp rosbag2_storage
  sensor_msgs nav_msgs geometry_msgs std_msgs visualization_msgs
  tf2 tf2_ros tf2_eigen tf2_geometry_msgs
  cv_bridge pcl_conversions angles
)

# ---- Bibliothek ----
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/utils.cpp
  src/${PROJECT_NAME}/pointnormal.cpp
  src/${PROJECT_NAME}/registration.cpp
  src/${PROJECT_NAME}/n_scan_normal.cpp
  src/${PROJECT_NAME}/odometrykeyframefuser.cpp
  src/${PROJECT_NAME}/eval_trajectory.cpp
  src/${PROJECT_NAME}/statistics.cpp
  src/${PROJECT_NAME}/lodestar.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPS})
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}
  Eigen3::Eigen OpenMP::OpenMP_CXX
)

# ---- Executable ----
add_executable(lodestar_odom src/lodestar_odom.cpp)
ament_target_dependencies(lodestar_odom ${ROS_DEPS})
target_link_libraries(lodestar_odom ${PROJECT_NAME} Boost::program_options)

# ---- Install ----
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS lodestar_odom
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_lodestar_odom.sh DESTINATION lib/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${ROS_DEPS} PCL OpenCV Ceres Eigen3)
ament_package()
```

Anmerkungen:
- `set(CMAKE_CXX_FLAGS ... -std=c++14)` und die `-O3`-Hacks aus dem alten File entfallen; `Release` setzt `-O3` automatisch, OpenMP kommt über das importierte Target `OpenMP::OpenMP_CXX`.
- Die Zeile `set(_include_dirs "include;/usr/include;/usr/include/opencv4")` war wirkungslos und wird gelöscht.
- `ament_target_dependencies` ist in Jazzy noch der Standard (Deprecation erst ab Kilted). Alternative moderne Form: `target_link_libraries(... rclcpp::rclcpp ${sensor_msgs_TARGETS} ...)`.

---

## 7. Schritt 3 – Zentrale Architekturentscheidung: Node-Handling

**Problem:** Unter ROS 1 erzeugte jede Klasse (`lodestar`, `OdometryKeyframeFuser`, `EvalTrajectory`, `Registration`, statisch `MapPointNormal::PublishMap`) ihr eigenes `ros::NodeHandle nh_("~")`. Unter ROS 2 gibt es kein globales Node-Objekt; Publisher/Broadcaster brauchen einen `rclcpp::Node`.

**Lösung (empfohlen, minimal-invasiv):**

1. `main()` erzeugt **genau einen** Node:
   ```cpp
   rclcpp::init(argc, argv);
   auto node = std::make_shared<rclcpp::Node>("lodestar_odom_node");
   ```
2. Der Node wird per `rclcpp::Node::SharedPtr` an alle Konstruktoren durchgereicht:
   - `lodestar(const Parameters&, rclcpp::Node::SharedPtr node, bool disable_callback)`
   - `OdometryKeyframeFuser(const Parameters&, rclcpp::Node::SharedPtr node, bool disable_callback)`
   - `EvalTrajectory(const Parameters&, rclcpp::Node::SharedPtr node, bool disable_callback)`
   - `Registration(rclcpp::Node::SharedPtr node)` und `n_scan_normal_reg(cost, loss, limit, opt, rclcpp::Node::SharedPtr node)`
3. `MapPointNormal` bekommt einen statischen Setter `static void SetNode(rclcpp::Node::SharedPtr n)` und ein statisches `rclcpp::Node::SharedPtr node_`; `main()` ruft `MapPointNormal::SetNode(node)` vor allem anderen auf. Wenn `node_` null ist, macht `PublishMap` nichts (Guard), statt abzustürzen.
4. Da das Programm offline läuft (Bag wird sequenziell gelesen, keine Subscriber-Callbacks nötig), ist **kein `rclcpp::spin`** erforderlich. Publisher senden auch ohne Spin. Für die (deaktivierten) Online-Subscriber müsste später ein Executor in einem Thread laufen (Abschnitt 13).
5. `GetParametersFromRos(ros::NodeHandle&)` in den drei `Parameters`-Klassen wird zu `GetParametersFromRos(rclcpp::Node& node)` mit `node.declare_parameter<T>("name", default)` + `node.get_parameter(...)`. Da `main()` heute ausschließlich `boost::program_options` benutzt, werden diese Methoden im Offline-Pfad nicht aufgerufen – sie müssen aber kompilieren.

**Typ-Aliase (in jeden Header übernehmen, spart Tipparbeit):**

```cpp
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using ImageConstPtr  = sensor_msgs::msg::Image::ConstSharedPtr;
```

---

## 8. Schritt 4 – Allgemeine API-Übersetzungstabelle

Diese Tabelle gilt für **alle** Dateien; die Datei-für-Datei-Anweisungen in Abschnitt 9 verweisen darauf.

### 8.1 Includes

| ROS 1 | ROS 2 Jazzy |
|---|---|
| `#include <ros/ros.h>`, `ros/node_handle.h`, `ros/time.h`, `ros/publisher.h`, `ros/subscriber.h` | `#include <rclcpp/rclcpp.hpp>` |
| `sensor_msgs/Image.h` | `sensor_msgs/msg/image.hpp` |
| `sensor_msgs/PointCloud2.h` | `sensor_msgs/msg/point_cloud2.hpp` |
| `sensor_msgs/image_encodings.h` | `sensor_msgs/image_encodings.hpp` |
| `nav_msgs/Odometry.h`, `nav_msgs/Path.h` | `nav_msgs/msg/odometry.hpp`, `nav_msgs/msg/path.hpp` |
| `geometry_msgs/PoseStamped.h`, `Transform.h`, `Point.h` | `geometry_msgs/msg/pose_stamped.hpp`, `transform_stamped.hpp`, `point.hpp` |
| `std_msgs/Header.h`, `Float32.h`, `ColorRGBA.h` | `std_msgs/msg/header.hpp`, `float32.hpp`, `color_rgba.hpp` |
| `std_msgs/Time.h` | löschen (`builtin_interfaces/msg/time.hpp` falls je nötig) |
| `visualization_msgs/Marker.h`, `MarkerArray.h` | `visualization_msgs/msg/marker.hpp`, `marker_array.hpp` |
| `cv_bridge/cv_bridge.h` | `cv_bridge/cv_bridge.hpp` (Jazzy; `.h` gibt Deprecation-Warnung) |
| `image_transport/image_transport.h` | löschen (ungenutzt) |
| `pcl_ros/point_cloud.h`, `pcl_ros/publisher.h`, `pcl_ros/transforms.h` | löschen; stattdessen `pcl_conversions/pcl_conversions.h` + `pcl/common/transforms.h` |
| `eigen_conversions/eigen_msg.h`, `tf_conversions/tf_eigen.h` | `tf2_eigen/tf2_eigen.hpp` |
| `tf/transform_broadcaster.h` | `tf2_ros/transform_broadcaster.h` |
| `tf/transform_listener.h`, `tf/message_filter.h` | löschen (ungenutzt) |
| `rosbag/bag.h`, `rosbag/view.h` | `rosbag2_cpp/reader.hpp`, `rosbag2_storage/storage_filter.hpp`, `rclcpp/serialization.hpp` |
| `boost/foreach.hpp` | löschen (range-for) |
| `boost/shared_ptr.hpp` | `<memory>` |
| `boost/filesystem.hpp` | `<filesystem>` |
| `laser_geometry/laser_geometry.h`, `message_filters/*` | löschen |
| `angles/angles.h` | unverändert (Paket `angles` existiert in ROS 2) |

### 8.2 Typen und Aufrufe

| ROS 1 | ROS 2 Jazzy |
|---|---|
| `ros::NodeHandle nh_("~")` | `rclcpp::Node::SharedPtr node_` (Konstruktorparameter) |
| `ros::Publisher pub = nh_.advertise<MsgT>(topic, q)` | `rclcpp::Publisher<MsgT>::SharedPtr pub = node_->create_publisher<MsgT>(topic, rclcpp::QoS(q))` |
| `nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>(...)` | `create_publisher<sensor_msgs::msg::PointCloud2>(...)` + `pcl::toROSMsg(cloud, msg)` vor `publish` |
| `pub.publish(msg)` | `pub->publish(msg)` (bei `SharedPtr`-Msgs: `pub->publish(*msg)`) |
| `nh_.subscribe<MsgT>(topic, q, &Cls::cb, this)` | `node_->create_subscription<MsgT>(topic, rclcpp::QoS(q), std::bind(&Cls::cb, this, std::placeholders::_1))` |
| `ros::TransportHints().tcpNoDelay(true)` | entfällt |
| `sensor_msgs::ImageConstPtr` | `sensor_msgs::msg::Image::ConstSharedPtr` |
| `nav_msgs::Odometry::ConstPtr` | `nav_msgs::msg::Odometry::ConstSharedPtr` |
| `sensor_msgs::ImagePtr` | `sensor_msgs::msg::Image::SharedPtr` |
| `MsgT` (z.B. `nav_msgs::Odometry`) | `MsgT` → `nav_msgs::msg::Odometry` (überall `::msg::` einfügen) |
| `ros::Time::now()` (für Header-Stamps) | `node_->now()` |
| `ros::Time::now()` (für Laufzeitmessung) | `std::chrono::steady_clock::now()` (besser, kein Node nötig) |
| `ros::Time t; t.toSec()` | `rclcpp::Time t; t.seconds()` |
| `ros::Duration d; d.toSec()`, `d.toNSec()` | `rclcpp::Duration d; d.seconds()`, `d.nanoseconds()` |
| `ros::Duration(0)` (Marker-Lifetime) | `rclcpp::Duration(0, 0)` – Marker-Feld ist `builtin_interfaces::msg::Duration`, Zuweisung: `m.lifetime = rclcpp::Duration(0,0);` funktioniert über impliziten Operator |
| `msg->header.stamp` (ist `ros::Time`) | ist `builtin_interfaces::msg::Time` → `rclcpp::Time(msg->header.stamp)` zum Rechnen |
| `pcl_conversions::toPCL(ros::Time, uint64_t&)` | `pcl_conversions::toPCL(const rclcpp::Time&, std::uint64_t&)` (existiert) |
| `pcl_conversions::fromPCL(uint64_t, ros::Time&)` | `pcl_conversions::fromPCL(const std::uint64_t&, rclcpp::Time&)` (existiert) |
| `pcl::fromROSMsg(*msg, cloud)` | unverändert (`pcl_conversions`) |
| `tf::poseEigenToMsg(T, pose)` | `pose = tf2::toMsg(T);` (`tf2_eigen`, `Eigen::Affine3d` → `geometry_msgs::msg::Pose`) |
| `tf::poseMsgToEigen(pose, T)` | `tf2::fromMsg(pose, T);` |
| `tf::transformEigenToTF(T, tf)` + `tf::StampedTransform` + `br.sendTransform(vec)` | `geometry_msgs::msg::TransformStamped ts = tf2::eigenToTransform(T); ts.header.stamp=…; ts.header.frame_id=…; ts.child_frame_id=…; br_->sendTransform(ts);` |
| `tf::TransformBroadcaster br;` | `std::unique_ptr<tf2_ros::TransformBroadcaster> br_;` initialisiert mit `std::make_unique<tf2_ros::TransformBroadcaster>(node_)` |
| `ros::ok()` | `rclcpp::ok()` |
| `ros::init(argc, argv, "name")` | `rclcpp::init(argc, argv); auto node = std::make_shared<rclcpp::Node>("name");` |
| Programmende | `rclcpp::shutdown();` |
| `boost::shared_ptr<T>` | `std::shared_ptr<T>` |
| `boost::filesystem::create_directories` | `std::filesystem::create_directories` |
| `BOOST_FOREACH(x, view)` | Range-for / `while(reader.has_next())` |
| `cv_bridge::toCvCopy(msg, enc)` | unverändert (nimmt `ConstSharedPtr`) |
| `cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg()` | `cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img).toImageMsg()` → gibt `sensor_msgs::msg::Image::SharedPtr` |

### 8.3 QoS

Alle Publisher aus ROS 1 hatten Queue-Größen 10–1000. Unter ROS 2 einfach `rclcpp::QoS(rclcpp::KeepLast(N))` mit demselben N (Standard: reliable, volatile). RViz2 abonniert PointCloud2/Odometry standardmäßig **reliable**, daher **nicht** `rclcpp::SensorDataQoS()` (best effort) nutzen, sonst zeigt RViz2 nichts.

---

## 9. Schritt 5 – Datei-für-Datei-Anweisungen

Reihenfolge ist so gewählt, dass jede Datei nur von bereits migrierten Dateien abhängt (Bottom-up). Nach jeder Datei `colcon build --packages-select lodestar_odometry` laufen lassen ist erst ab 9.9 sinnvoll; bis dahin Fehler in noch nicht migrierten Dateien ignorieren oder die Sources im CMake temporär auskommentieren.

### 9.1 `statistics.h` / `statistics.cpp`

**Header:**
- `#include <ros/ros.h>` → `#include <rclcpp/rclcpp.hpp>` und `#include <chrono>`.
- Signatur ändern: `double ToMs(const ros::Duration& dur);` → 
  ```cpp
  double ToMs(const rclcpp::Duration& dur);
  double ToMs(const std::chrono::steady_clock::duration& dur);   // neu, für Laufzeitmessung
  ```

**Source:**
```cpp
double ToMs(const rclcpp::Duration& dur){ return dur.nanoseconds()/1000000.0; }
double ToMs(const std::chrono::steady_clock::duration& dur){
  return std::chrono::duration<double, std::milli>(dur).count();
}
```
Rest unverändert.

### 9.2 `utils.h` / `utils.cpp`

- Löschen: `pcl_ros/point_cloud.h`, `pcl_ros/publisher.h`, `ros/ros.h`.
- Hinzufügen: `#include <rclcpp/rclcpp.hpp>` (nur falls später benötigt; aktuell reicht `sensor_msgs/msg/point_cloud2.hpp`).
- `sensor_msgs/PointCloud2.h` → `sensor_msgs/msg/point_cloud2.hpp`.
- Code in `utils.cpp` ist reines Eigen/PCL/Ceres → **keine Änderung**.

### 9.3 `pointnormal.h` / `pointnormal.cpp`

**Header:**
- Includes gemäß 8.1: `pcl_ros/*` raus, `ros/ros.h` → `rclcpp/rclcpp.hpp`, `visualization_msgs/MarkerArray.h` → `visualization_msgs/msg/marker_array.hpp`, `boost/shared_ptr.hpp` → `<memory>`, zusätzlich `geometry_msgs/msg/point.hpp`.
- `typedef boost::shared_ptr<cell> cellptr;` → `using cellptr = std::shared_ptr<cell>;`
- `typedef boost::shared_ptr<MapPointNormal> MapNormalPtr;` → `using MapNormalPtr = std::shared_ptr<MapPointNormal>;`
- `ros::Time GetTime()` → 
  ```cpp
  rclcpp::Time GetTime(){ rclcpp::Time t; pcl_conversions::fromPCL(input_->header.stamp, t); return t; }
  ```
- Statische Member ersetzen:
  ```cpp
  static std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> pubs;
  static rclcpp::Node::SharedPtr node_;
  static void SetNode(rclcpp::Node::SharedPtr n){ node_ = n; }
  ```
- Freie Funktionen: `visualization_msgs::Marker DefaultMarker(const ros::Time&, ...)` → `visualization_msgs::msg::Marker DefaultMarker(const rclcpp::Time&, ...)`; analog `Cells2Markers(..., const rclcpp::Time&, ...)` mit Rückgabetyp `visualization_msgs::msg::MarkerArray`; `geometry_msgs::Point Pntgeom(...)` → `geometry_msgs::msg::Point`.

**Source:**
- Zeile 4: `std::map<std::string,ros::Publisher> MapPointNormal::pubs;` → 
  ```cpp
  std::map<std::string, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> MapPointNormal::pubs;
  rclcpp::Node::SharedPtr MapPointNormal::node_ = nullptr;
  ```
- `DefaultMarker`: `marker.lifetime = ros::Duration(0.0);` → `marker.lifetime = rclcpp::Duration(0, 0);`; `visualization_msgs::Marker::ARROW` → `visualization_msgs::msg::Marker::ARROW` (analog `ADD`, `DELETEALL`, `TEXT_VIEW_FACING`, `LINE_LIST`).
- `PublishDataAssociationsMap` und `PublishMap` (Zeilen ~420–555), jeweils der Publisher-Lookup:
  ```cpp
  if(!node_) return;                                   // Guard
  auto it = MapPointNormal::pubs.find(topic);
  if (it == pubs.end()){
    pubs[topic] = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, rclcpp::QoS(100));
    it = MapPointNormal::pubs.find(topic);
  }
  ...
  it->second->publish(marr_delete);   // -> statt .
  ```
- `ros::Time t = ros::Time::now();` → `rclcpp::Time t = node_->now();` (in beiden Funktionen; auch beim Aufruf `Cells2Markers(cells, node_->now(), ...)`).
- Hinweis: Das Topic `"/current_normals"` wird von `OdometryKeyframeFuser::processFrame` als absoluter Name übergeben; unter ROS 2 sind absolute Namen in `create_publisher` erlaubt → bleibt.

### 9.4 `registration.h` / `registration.cpp`

**Header:**
- Includes gemäß 8.1 (`pcl_ros/*` raus, `ros/ros.h` → `rclcpp`, `visualization_msgs/msg/marker_array.hpp`, `<memory>`).
- `typedef boost::shared_ptr<lodestar_odom::Registration> regPtr;` → `using regPtr = std::shared_ptr<Registration>;`
- Member: 
  ```cpp
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ceres::Problem> problem_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_association;
  ```
- Konstruktor: `Registration(rclcpp::Node::SharedPtr node = nullptr);`

**Source:**
```cpp
Registration::Registration(rclcpp::Node::SharedPtr node) : node_(node) {
  this->options_.line_search_direction_type = ceres::LineSearchDirectionType::BFGS;
  ...
  if(node_)
    pub_association = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/associations", rclcpp::QoS(100));
}
```
Ceres-Code (`GetLoss`, Loss-Typen) unverändert.

### 9.5 `n_scan_normal.h` / `n_scan_normal.cpp`

- Includes: `pcl_ros/*` raus, `ros/ros.h` → `rclcpp/rclcpp.hpp`, `sensor_msgs/PointCloud2.h` → `.../msg/point_cloud2.hpp`.
- Konstruktoren um Node-Parameter erweitern und an `Registration` durchreichen:
  ```cpp
  n_scan_normal_reg(rclcpp::Node::SharedPtr node = nullptr);
  n_scan_normal_reg(const cost_metric& cost, loss_type loss = Huber, double loss_limit = 0.1,
                    const weightoption opt = weightoption::Uniform, rclcpp::Node::SharedPtr node = nullptr);
  ```
  In der `.cpp`: `: Registration(node)` in der Initialisiererliste.
- `problem_ = boost::shared_ptr<ceres::Problem>(new ceres::Problem());` (Zeile ~230) → `problem_ = std::make_shared<ceres::Problem>();`
- Auskommentierte `ros::Time`-Zeile (238) löschen.
- Ceres-Costfunctions (`AutoDiffCostFunction`, `ceres::cos/sin`) bleiben unverändert (Ceres 2.2-kompatibel; `ceres/manifold.h` existiert).

### 9.6 `lodestar.h` / `lodestar.cpp`

**Header – Includes neu:**
```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Eigen>
#include <chrono>
#include "lodestar_odometry/statistics.h"
#include "lodestar_odometry/pointnormal.h"
```
Entfernt: `pcl_ros/*`, `image_transport`, `ros/*`, `eigen_conversions`, `tf_conversions`, `tf/*`.

**Header – `Parameters::GetParametersFromRos`:**
```cpp
void GetParametersFromRos(rclcpp::Node& n){
  range_res         = n.declare_parameter<double>("range_res", 0.0438);
  z_min             = n.declare_parameter<double>("z_min", 60.0);
  min_distance      = n.declare_parameter<double>("min_distance", 2.5);
  max_distance      = n.declare_parameter<double>("max_distance", 130.0);
  topic_filtered    = n.declare_parameter<std::string>("topic_filtered", "/marine/Filtered");
  radar_frameid     = n.declare_parameter<std::string>("radar_frameid", "sensor_est");
  dataset           = n.declare_parameter<std::string>("dataset", "marine");
  radar_topic       = n.declare_parameter<std::string>("radar_topic_name", "/radar_data");
  contour_threshold = n.declare_parameter<int>("contour_threshold", 50);
  k_nearest         = n.declare_parameter<int>("k_nearest", 20);
}
```
(`float`-Member bekommen `double`-Parameter; rclcpp kennt keinen `float`-Parametertyp.)

**Header – Klasse:**
- Konstruktor: `lodestar(const Parameters& pars, rclcpp::Node::SharedPtr node, bool disable_callback = false);`
- `CallbackOffline(const sensor_msgs::msg::Image::ConstSharedPtr& img, ...)`, `Callback(const sensor_msgs::msg::Image::ConstSharedPtr& img)`, `CallbackOxford(...)` analog.
- Member ersetzen:
  ```cpp
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr FilteredPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPublisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rotPublisher;
  ```
  Löschen: `ExperimentalPublisher`, `UnfilteredPublisher`, `image_transport::Publisher pub`, `image_transport::ImageTransport it`, `nh_`.

**Source – Konstruktor:**
```cpp
lodestar::lodestar(const Parameters& pars, rclcpp::Node::SharedPtr node, bool disable_callback)
  : par(pars), node_(node) {
  min_distance_sqrd = par.min_distance*par.min_distance;
  FilteredPublisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(par.topic_filtered, rclcpp::QoS(1000));
  imgPublisher      = node_->create_publisher<sensor_msgs::msg::Image>("/radar_imported", rclcpp::QoS(1000));
  rotPublisher      = node_->create_publisher<nav_msgs::msg::Odometry>("/rot_lodestar", rclcpp::QoS(1000));
  if(!disable_callback){
    sub = node_->create_subscription<sensor_msgs::msg::Image>(
        pars.radar_topic, rclcpp::QoS(1000),
        std::bind(&lodestar::Callback, this, std::placeholders::_1));
  }
}
```

**Source – `Callback`:**
- `if(marine_radar_img==NULL)` → `if(!marine_radar_img)`.
- Alle Zeitmessungen (`t0`, `lodestar_start/end`, `contour_start/end`, `k_nearest_start/end`, `t2`) auf `std::chrono::steady_clock::now()` umstellen; Ausgabe z.B.
  ```cpp
  auto lodestar_start = std::chrono::steady_clock::now();
  ...
  cout << "Lodestar Computation time == " << ToMs(std::chrono::steady_clock::now()-lodestar_start) << " ms" << endl;
  ```
  und am Ende `lodestar_odom::timing.Document("Filtering", ToMs(std::chrono::steady_clock::now()-t0));`.
- `cv_bridge::toCvCopy(marine_radar_img, sensor_msgs::image_encodings::MONO8)` bleibt.
- Zeitstempel-Logik:
  ```cpp
  rclcpp::Time msg_stamp(marine_radar_img->header.stamp);
  rclcpp::Time tstamp = msg_stamp.seconds() < 0.001 ? node_->now() : msg_stamp;
  pcl_conversions::toPCL(tstamp, cloud_filtered->header.stamp);
  ```
- PointCloud publizieren:
  ```cpp
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_filtered, cloud_msg);
  cloud_msg.header.frame_id = par.radar_frameid;
  cloud_msg.header.stamp = tstamp;
  FilteredPublisher->publish(cloud_msg);
  ```
- Bild publizieren:
  ```cpp
  auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img).toImageMsg();
  imgPublisher->publish(*img_msg);
  ```
- Rotation-Odometry:
  ```cpp
  nav_msgs::msg::Odometry rot_msg;
  ...
  rot_msg.header.stamp = node_->now();
  rot_msg.header.frame_id = "world";
  rot_msg.child_frame_id = "sensor";
  rot_msg.pose.pose = tf2::toMsg(Trot);
  rotPublisher->publish(rot_msg);
  ```
- `polar_transform`, `crossCorrelation`, `polarToNearPol`, `toPointCloud`, `rotationCorrection`: **unverändert** (reines OpenCV/PCL).
- `CallbackOffline`: nur Signatur (`ConstSharedPtr`) ändern.

### 9.7 `odometrykeyframefuser.h` / `odometrykeyframefuser.cpp`

**Header – Includes neu (nur was gebraucht wird):**
```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <boost/circular_buffer.hpp>
#include <memory>
#include <chrono>
#include <fstream>
#include "lodestar_odometry/lodestar.h"
#include "lodestar_odometry/utils.h"
#include "lodestar_odometry/pointnormal.h"
#include "lodestar_odometry/n_scan_normal.h"
#include "lodestar_odometry/statistics.h"
```
Entfernt: `laser_geometry`, `message_filters`, `tf/*`, `pcl_ros/*`, `std_msgs/Time.h`, `eigen_conversions`, `tf_conversions`, `boost/shared_ptr.hpp`.

**Header – Deklarationen:**
- `visualization_msgs::Marker GetDefault();` → `visualization_msgs::msg::Marker GetDefault(rclcpp::Node::SharedPtr node);` (braucht `now()`), oder Stamp auf 0 lassen und Node-Parameter weglassen. Funktion wird aktuell nirgends aufgerufen → einfachste Lösung: Stamp weglassen (`m.header.stamp = rclcpp::Time(0);`).
- `GetParametersFromRos(rclcpp::Node& n)` analog 9.6 mit `declare_parameter` (alle Bool-/Double-/String-/Int-Parameter; `downsample_factor` → `MapPointNormal::downsample_factor = n.declare_parameter<double>("downsample_factor", 1.0);`).
- Member:
  ```cpp
  std::shared_ptr<n_scan_normal_reg> radar_reg = nullptr;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_callback;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_rot_est;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_current_publisher, pose_keyframe_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubsrc_cloud_latest, pub_cloud_keyframe;
  std::unique_ptr<tf2_ros::TransformBroadcaster> Tbr;
  ```
- Konstruktor: `OdometryKeyframeFuser(const Parameters& pars, rclcpp::Node::SharedPtr node, bool disable_callback = false);`
- `FormatOdomMsg(..., const rclcpp::Time& t, Matrix6d& Cov)` → Rückgabetyp `nav_msgs::msg::Odometry`.
- `CallbackRot(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)`, `pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_in)`.

**Source – Konstruktor:**
```cpp
OdometryKeyframeFuser::OdometryKeyframeFuser(const Parameters& pars, rclcpp::Node::SharedPtr node, bool disable_callback)
  : par(pars), node_(node) {
  assert(...);  // wie bisher
  radar_reg = std::make_shared<n_scan_normal_reg>(Str2Cost(par.cost_type), Str2loss(par.loss_type_),
                                                  par.loss_limit_, par.weight_opt, node_);
  radar_reg->SetD2dPar(par.covar_scale_, par.regularization_);
  Tprev_fused = Tcurrent = T_prev = Tmot = Eigen::Affine3d::Identity();

  pose_current_publisher  = node_->create_publisher<nav_msgs::msg::Odometry>("~/" + par.odom_latest_topic, rclcpp::QoS(50));
  pose_keyframe_publisher = node_->create_publisher<nav_msgs::msg::Odometry>("~/" + par.odom_keyframe_topic, rclcpp::QoS(50));
  pubsrc_cloud_latest     = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/" + par.scan_registered_latest_topic, rclcpp::QoS(1000));
  pub_cloud_keyframe      = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/" + par.scan_registered_keyframe_topic, rclcpp::QoS(1000));
  Tbr = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  sub_rot_est = node_->create_subscription<nav_msgs::msg::Odometry>("/rot_lodestar", rclcpp::QoS(1000),
                  std::bind(&OdometryKeyframeFuser::CallbackRot, this, std::placeholders::_1));
  if(!disable_callback){
    pointcloud_callback = node_->create_subscription<sensor_msgs::msg::PointCloud2>(par.input_points_topic, rclcpp::QoS(1000),
        std::bind(static_cast<void (OdometryKeyframeFuser::*)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)>(&OdometryKeyframeFuser::pointcloudCallback), this, std::placeholders::_1));
  }
}
```
Achtung: Die Default-Topic-Strings in `Parameters` (`"radar_odom"` usw.) haben **keinen** führenden `/`; das `"~/"`-Präfix stellt den ROS-1-Effektivnamen `/lodestar_odom_node/radar_odom` wieder her. Falls ein Nutzer im YAML einen absoluten Namen setzt, das Präfix nur anhängen, wenn der String nicht mit `/` beginnt (kleine Hilfsfunktion `PrivateTopic(const std::string&)`).

**Source – `FormatOdomMsg`:**
```cpp
odom_msg.header.stamp = t;                         // rclcpp::Time -> builtin_interfaces::msg::Time implizit
odom_msg.header.frame_id = par.odometry_link_id;
odom_msg.child_frame_id = "sensor";
odom_msg.pose.pose = tf2::toMsg(T);
```

**Source – `processFrame`:**
- Zeitmessung `t0..t4` auf `std::chrono::steady_clock`.
- Zeitstempel: `rclcpp::Time t; pcl_conversions::fromPCL(cloud->header.stamp, t);`
- PointCloud-Publish:
  ```cpp
  auto ToMsg = [&](pcl::PointCloud<pcl::PointXYZI>& c){
    sensor_msgs::msg::PointCloud2 m; pcl::toROSMsg(c, m); return m; };
  pubsrc_cloud_latest->publish(ToMsg(cld_latest));
  pose_current_publisher->publish(msg_current);
  ```
  (`FormatScanMsg` setzt `frame_id`/`stamp` weiterhin im PCL-Header; `pcl::toROSMsg` übernimmt beides.)
- TF:
  ```cpp
  if(par.publish_tf_){
    geometry_msgs::msg::TransformStamped ts = tf2::eigenToTransform(Tcurrent);
    ts.header.stamp = t;
    ts.header.frame_id = par.odometry_link_id;
    ts.child_frame_id = "radar_link";
    Tbr->sendTransform(ts);
  }
  ```
- Keyframe-Zweig analog (`pub_cloud_keyframe->publish(ToMsg(cld_keyframe)); pose_keyframe_publisher->publish(msg_keyframe);`).

**Source – `pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)`:**
```cpp
pcl::fromROSMsg(*msg_in, *cloud);
pcl_conversions::toPCL(rclcpp::Time(msg_in->header.stamp), cloud->header.stamp);
```
Die beiden anderen Overloads: nur Zeitmessung anpassen.

**Source – `GetDefault`:** `m.lifetime = rclcpp::Duration(0,0); m.header.stamp = rclcpp::Time(0);` und `visualization_msgs::msg::Marker::LINE_LIST/ADD`.

### 9.8 `eval_trajectory.h` / `eval_trajectory.cpp`

**Header – Includes neu:**
```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <filesystem>
#include <memory>
```
Entfernt: `message_filters/*`, `boost/filesystem.hpp`, `tf/*`, `eigen_conversions`, `tf_conversions`, `pcl_ros/*`, `pcl/2d/convolution.h`, `pcl/filters/random_sample.h`, `pcl/filters/radius_outlier_removal.h` (ungenutzt), `using namespace message_filters; using namespace sensor_msgs;`.

**Header – Typen:**
- `typedef std::pair<Eigen::Affine3d, ros::Time> poseStamped;` → `using poseStamped = std::pair<Eigen::Affine3d, rclcpp::Time>;`
- `typedef sync_policies::ApproximateTime<...> double_odom;` löschen; ebenso `pose_sub_est`, `rot_sub_est`, `sync`.
- Member:
  ```cpp
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_est;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_rot_est;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_est;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br;
  ```
- Signaturen: `EvalTrajectory(const Parameters&, rclcpp::Node::SharedPtr node, bool disable_callback=false)`, `CallbackEst(const nav_msgs::msg::Odometry::ConstSharedPtr&)`, `CallbackRot(const std_msgs::msg::Float32::ConstSharedPtr&)`, `PublishTrajectory(poseStampedVector&, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr)`, `FindElement(const rclcpp::Time&)`.
- `GetParametersFromRos(rclcpp::Node& n)` mit `declare_parameter` (Namen: `est_topic`, `est_output_dir`, `bag_name`, `method`, `save_pcd`, `synced_callback`).

**Source:**
- Konstruktor:
  ```cpp
  EvalTrajectory::EvalTrajectory(const Parameters& pars, rclcpp::Node::SharedPtr node, bool disable_callback)
    : par(pars), node_(node), downsampled(new pcl::PointCloud<pcl::PointXYZI>()) {
    if(!disable_callback && !par.synced_callback){
      sub_rot_est = node_->create_subscription<std_msgs::msg::Float32>("/rot_lodestar", rclcpp::QoS(1000),
                      std::bind(&EvalTrajectory::CallbackRot, this, std::placeholders::_1));
      sub_est = node_->create_subscription<nav_msgs::msg::Odometry>(par.odom_est_topic, rclcpp::QoS(1000),
                      std::bind(&EvalTrajectory::CallbackEst, this, std::placeholders::_1));
    }
    pub_est   = node_->create_publisher<nav_msgs::msg::Path>("~/path_est", rclcpp::QoS(10));
    pub_cloud = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/map_cloud", rclcpp::QoS(10));
    br = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }
  ```
- `CallbackEst`: `tf2::fromMsg(msg->pose.pose, T); rclcpp::Time t(msg->header.stamp);`
- `CallbackRot`: `rclcpp::Time t = node_->now();`
- `PublishTrajectory`: `path.header.stamp = node_->now();`, `Tstamped.pose = tf2::toMsg(T);`, `pub->publish(path);`; Zeile `std::vector<tf::StampedTransform> trans_vek;` löschen.
- `Save`: `std::filesystem::create_directories(par.est_output_dir);`
- `Write`, `SavePCD`, `DatasetToSequence`, `CallbackESTEigen`: **unverändert** (inkl. des hart kodierten Skalenfaktors `22.73` – der ist ein Algorithmus-/Dataset-Thema, nicht Teil der Migration; siehe Hinweis in 13).

### 9.9 `lodestar_odom.cpp` (main, Bag-Reader)

Diese Datei wird am stärksten umgebaut. Vollständige Zielfassung der ROS-relevanten Teile:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "lodestar_odometry/lodestar.h"
#include "lodestar_odometry/odometrykeyframefuser.h"
#include "lodestar_odometry/eval_trajectory.h"

using namespace lodestar_odom;
namespace po = boost::program_options;

typedef struct eval_parameters_{
  std::string bag_file_path = "";
  bool save_pcds = false;
} eval_parameters;

class radarReader {
  rclcpp::Node::SharedPtr node_;
  EvalTrajectory eval;
  lodestar driver;
  OdometryKeyframeFuser fuser;
public:
  radarReader(rclcpp::Node::SharedPtr node,
              const OdometryKeyframeFuser::Parameters& odom_pars,
              const lodestar::Parameters& rad_pars,
              const EvalTrajectory::Parameters& eval_par,
              const eval_parameters& p)
    : node_(node), eval(eval_par, node, true), driver(rad_pars, node, true), fuser(odom_pars, node, true) {

    std::cout << "Loading bag from: " << p.bag_file_path << std::endl;

    rosbag2_cpp::Reader reader;
    reader.open(p.bag_file_path);                       // Ordner mit metadata.yaml ODER .mcap/.db3-Datei
    rosbag2_storage::StorageFilter filter;
    filter.topics = { rad_pars.radar_topic };
    reader.set_filter(filter);

    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    int frame = 0;
    std::chrono::steady_clock::duration tot{0};

    while (reader.has_next() && rclcpp::ok()) {
      auto bag_msg = reader.read_next();
      if (bag_msg->topic_name != rad_pars.radar_topic) continue;

      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
      serializer.deserialize_message(&serialized, image_msg.get());

      auto tinit = std::chrono::steady_clock::now();
      pcl::PointCloud<pcl::PointXYZI>::Ptr lodestar_cloud(new pcl::PointCloud<pcl::PointXYZI>());
      Eigen::Affine3d Trot; Eigen::Vector3d dense_trans;
      driver.CallbackOffline(image_msg, lodestar_cloud, Trot, dense_trans);
      Eigen::Matrix3d rotMat = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      Trot.rotate(rotMat);

      lodestar_odom::timing.Document("Filtered points", lodestar_cloud->size());
      Eigen::Affine3d Tcurrent;
      fuser.pointcloudCallback(lodestar_cloud, Tcurrent, Trot);
      Eigen::Affine3d T_final = Tcurrent*Trot;

      const rclcpp::Time t(image_msg->header.stamp);
      if(eval_par.save_pcd) eval.CallbackESTEigen(std::make_pair(T_final, t), *lodestar_cloud);
      else                  eval.CallbackESTEigen(std::make_pair(T_final, t));

      auto d = std::chrono::steady_clock::now() - tinit;
      tot += d;
      double tot_s = std::chrono::duration<double>(tot).count();
      std::cout << "Frame: " << frame << ", Odom duration: " << std::chrono::duration<double>(d).count()
                << " sec, avg: " << ++frame/tot_s << " Hz" << std::endl;
    }
    std::cout << fuser.GetStatus() << std::endl;
    reader.close();
  }
  void Save(){ eval.Save(); }
  size_t GetSize(){ return eval.GetSize(); }
};

// ReadOptions(...) bleibt inhaltlich identisch (boost::program_options)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lodestar_odom_node");
  MapPointNormal::SetNode(node);

  // ROS-2-Argumente (--ros-args ...) entfernen, damit program_options nicht stolpert
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> cargs; for(auto& a : args) cargs.push_back(a.data());

  OdometryKeyframeFuser::Parameters odom_pars; lodestar::Parameters rad_pars;
  EvalTrajectory::Parameters eval_pars; eval_parameters eval_p;
  ReadOptions(static_cast<int>(cargs.size()), cargs.data(), odom_pars, rad_pars, eval_pars, eval_p);

  // pars.txt vorher/nachher: unverändert
  ...
  radarReader reader(node, odom_pars, rad_pars, eval_pars, eval_p);
  reader.Save();
  ...
  rclcpp::shutdown();
  return 0;
}
```

Wichtige Punkte:
- **Member-Initialisierungsreihenfolge:** In der Klasse sind `eval`, `driver`, `fuser` in dieser Reihenfolge deklariert; die Initialisiererliste muss dieselbe Reihenfolge haben (sonst `-Wreorder`). Im Original war das schon inkonsistent.
- `#define foreach BOOST_FOREACH`, `#define MAX_SIZE 3`, `ros::Duration tot` entfallen.
- `rclcpp::remove_ros_arguments` ist wichtig, weil `ros2 run pkg exe --ros-args -p x:=y` sonst von `parse_command_line` als unbekannte Option abgelehnt würde.
- `reader.open()` akzeptiert in Jazzy sowohl einen Bag-**Ordner** (mit `metadata.yaml`) als auch direkt eine `.mcap`-/`.db3`-Datei. Das Startskript übergibt den Ordner.

---

## 10. Schritt 6 – Launch, RViz, Startskript

### 10.1 `launch/vis.launch.py` (ersetzt `vis.launch`)

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    share = get_package_share_directory('lodestar_odometry')
    eval_arg = DeclareLaunchArgument('eval', default_value='false')
    rviz_cfg = PythonExpression([
        "'", os.path.join(share, 'rviz', 'odom_eval.rviz'), "' if '", LaunchConfiguration('eval'),
        "' == 'true' else '", os.path.join(share, 'rviz', 'odom.rviz'), "'"])
    return LaunchDescription([
        eval_arg,
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='screen'),
    ])
```
(`odom_eval.rviz` existiert im Repo nicht – entweder anlegen oder den `eval`-Zweig streichen.)

### 10.2 `launch/lodestar_odom.launch.py` (optional, komfortabel)

Startet rviz2 und den Node zusammen; CLI-Parameter werden als `arguments` durchgereicht:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('lodestar_odometry')
    bag = LaunchConfiguration('bag_path'); est = LaunchConfiguration('est_directory')
    seq = LaunchConfiguration('sequence')
    return LaunchDescription([
        DeclareLaunchArgument('bag_path'), DeclareLaunchArgument('est_directory'),
        DeclareLaunchArgument('sequence', default_value='test_sequence3'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'vis.launch.py'))),
        Node(package='lodestar_odometry', executable='lodestar_odom', name='lodestar_odom_node', output='screen',
             arguments=['--bag_path', bag, '--est_directory', est, '--sequence', seq,
                        '--range-res', '0.05', '--cost_type', 'P2P', '--submap_scan_size', '1',
                        '--registered_min_keyframe_dist', '1.5', '--res', '3', '--z-min', '60',
                        '--weight_option', '4', '--weight_intensity', 'true', '--soft_constraint', 'false',
                        '--disable_compensate', 'true', '--job_nr', '1', '--dataset', 'marine',
                        '--radar_topic', '/radar_image_inrange', '--contour_threshold', '214', '--k_nearest', '20']),
    ])
```

### 10.3 `scripts/run_lodestar_odom.sh` (ersetzt `launch/run_lodestar_odom`)

```bash
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

SEQUENCE="test_sequence3"
BAG_BASE_PATH="$HOME/ros2_data/odom_test"              # ROS-2-Bags (Ordner!)
BAG_FILE_PATH="${BAG_BASE_PATH}/${SEQUENCE}"           # Ordner mit metadata.yaml
EVAL_BASE_DIR="$HOME/ros2_data/odom_test/${SEQUENCE}/eval"
est_dir="${EVAL_BASE_DIR}/"
mkdir -p "${est_dir}"

# Parameter (unverändert gegenüber ROS 1)
cost_type="P2P"; submap_scan_size="1"; registered_min_keyframe_dist="1.5"; res="3"; zmin="60"
weight_option="4"; weight_intensity="true"; range_resolution="0.05"; soft_constraint="false"; disable_compensate="true"
radar_topic="/radar_image_inrange"; contour_threshold="214"; k_nearest="20"

pars="--range-res ${range_resolution} --sequence ${SEQUENCE} --soft_constraint ${soft_constraint} \
 --disable_compensate ${disable_compensate} --cost_type ${cost_type} --submap_scan_size ${submap_scan_size} \
 --registered_min_keyframe_dist ${registered_min_keyframe_dist} --res ${res} --bag_path ${BAG_FILE_PATH} \
 --est_directory ${est_dir} --job_nr 1 --z-min ${zmin} --weight_option ${weight_option} \
 --weight_intensity ${weight_intensity} --dataset marine --radar_topic ${radar_topic} \
 --contour_threshold ${contour_threshold} --k_nearest ${k_nearest}"

ros2 launch lodestar_odometry vis.launch.py &
RVIZ_PID=$!
sleep 2
ros2 run lodestar_odometry lodestar_odom ${pars}
kill $RVIZ_PID 2>/dev/null || true
```
`chmod +x scripts/run_lodestar_odom.sh`. Kein `roscore` mehr nötig. `killall rviz` → entfällt (oder `rviz2`).

### 10.4 `rviz/odom.rviz` konvertieren

Zwei Wege:

**A) Empfohlen – in RViz2 neu speichern:** `rviz2` starten, Fixed Frame `world`, Displays anlegen:
- `PointCloud2` auf `/lodestar_odom_node/radar_registered` (2× wie im Original, unterschiedliche Farbgebung),
- `Odometry` auf `/lodestar_odom_node/radar_odom` (Shape Arrow, Covariance an),
- `Grid`, `TF`,
dann *File → Save Config As* → `rviz/odom.rviz`.

**B) Manuell editieren** (Mapping der wichtigsten Felder):

| ROS 1 (`odom.rviz`) | ROS 2 |
|---|---|
| `Class: rviz/Displays` / `rviz/Selection` / `rviz/Tool Properties` / `rviz/Views` / `rviz/Time` | `rviz_common/Displays`, `rviz_common/Selection`, `rviz_common/Tool Properties`, `rviz_common/Views`, `rviz_common/Time` |
| `Class: rviz/Grid`, `rviz/PointCloud2`, `rviz/Odometry` | `rviz_default_plugins/Grid`, `rviz_default_plugins/PointCloud2`, `rviz_default_plugins/Odometry` |
| `Topic: /lodestar_odom_node/radar_registered` | `Topic:` `  Depth: 5` `  Durability Policy: Volatile` `  History Policy: Keep Last` `  Reliability Policy: Reliable` `  Value: /lodestar_odom_node/radar_registered` (verschachteltes Mapping; bei PointCloud2 zusätzlich `Filter size: 10`) |
| Tools `rviz/Interact` … `rviz/PublishPoint` | `rviz_default_plugins/Interact`, `MoveCamera`, `Select`, `FocusCamera`, `Measure`, `SetInitialPose`, `SetGoal`, `PublishPoint` |
| `Class: rviz/ThirdPersonFollower` (View) | `rviz_default_plugins/ThirdPersonFollower` |
| `Visualization Manager: … Global Options: Fixed Frame: world` | unverändert |
| Block `Enabled Statistics`/`Unreliable` | löschen |

Nach Weg B unbedingt einmal `rviz2 -d rviz/odom.rviz` laden und mit *Save Config* neu schreiben, damit alle Felder das aktuelle Schema haben.

### 10.5 `README.md` aktualisieren

Abschnitte *Prerequisites*, *Build with catkin*, *Running* ersetzen durch: Ubuntu 24.04 + Jazzy, `colcon build`, `source install/setup.bash`, Bag-Konvertierung (Abschnitt 11), `ros2 run lodestar_odometry lodestar_odom …` bzw. `scripts/run_lodestar_odom.sh`. Hinweis auf Topic `/radar_image_inrange` und den Pohang-RCS-Inversion-Kommentar aus dem alten Skript übernehmen.

---

## 11. Schritt 7 – Bag-Dateien konvertieren (ROS 1 `.bag` → ROS 2)

rosbag2 kann ROS-1-Bags nicht lesen. Konvertierung mit dem Python-Tool `rosbags` (kein ROS 1 nötig):

```bash
pip3 install --user rosbags
# Standard (SQLite3, ROS-2-Typen automatisch: sensor_msgs/Image -> sensor_msgs/msg/Image)
rosbags-convert --src ~/ros1_data/odom_test/test_sequence3.bag --dst ~/ros2_data/odom_test/test_sequence3
# Optional: nur das Radar-Topic mitnehmen (kleinere Bags)
rosbags-convert --src ... --dst ... --include-topic /radar_image_inrange
# Optional: MCAP statt SQLite (Jazzy-Standard)
rosbags-convert --src ... --dst ... --dst-storage mcap
```

Ergebnis ist ein **Ordner** `test_sequence3/` mit `metadata.yaml` + `test_sequence3_0.db3` (oder `.mcap`). Dieser Ordnerpfad wird als `--bag_path` übergeben.

Prüfen:

```bash
ros2 bag info ~/ros2_data/odom_test/test_sequence3
# erwartet: Topic /radar_image_inrange | Type sensor_msgs/msg/Image | Count N
```

Encoding-Check (LodeStar erwartet `mono8`): `ros2 topic echo --once /radar_image_inrange --field encoding` während `ros2 bag play`.

Für den Pohang-Canal-Datensatz: Bag zuerst konvertieren, danach ggf. Topic-Name per `--radar_topic` anpassen; die im alten Skript erwähnte RCS-Inversion bleibt eine Codeänderung in `lodestar.cpp` (unabhängig von der Migration).

---

## 12. Schritt 8 – Build, Test, Verifikation

### 12.1 Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y     # zieht alle <depend> aus package.xml
colcon build --packages-select lodestar_odometry --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Ziel: **0 Errors**. Warnungen zu `cv::linearPolar` (deprecated) und `-Wsign-compare` sind akzeptabel.

### 12.2 Smoke-Tests

```bash
# 1) Executable existiert und startet
ros2 run lodestar_odometry lodestar_odom --help

# 2) Node-Interface prüfen (in 2. Terminal während eines Laufs)
ros2 node list                      # /lodestar_odom_node
ros2 topic list                     # /lodestar_odom_node/radar_odom, /lodestar_odom_node/radar_registered,
                                    # /marine/Filtered, /rot_lodestar, /radar_imported, /current_normals, /tf
ros2 topic hz /lodestar_odom_node/radar_odom
ros2 run tf2_tools view_frames      # world -> radar_link

# 3) Kompletter Lauf
scripts/run_lodestar_odom.sh
```

### 12.3 Funktionale Verifikation (Regression gegen ROS 1)

1. Denselben Bag einmal mit dem ROS-1-Build (20.04-Rechner/Docker `ros:noetic`) und einmal mit dem ROS-2-Build laufen lassen.
2. Beide `01.txt` (KITTI-Format) mit `evo` vergleichen:
   ```bash
   pip install evo
   evo_traj kitti ros1/01.txt ros2/01.txt --plot --align
   evo_ape kitti ros1/01.txt ros2/01.txt
   ```
   Erwartung: identische Pose-Anzahl, APE ≈ 0 (Abweichungen nur durch Ceres-Version/Fließkomma im Bereich < 1e-3 m).
3. `pars.txt` vergleichen (Parameter-Echo muss identisch sein, `nr_frames` gleich).
4. RViz2: Punktwolke und Odometrie-Pfeile erscheinen im Frame `world`.

### 12.4 Typische Compile-Fehler und ihre Ursache

| Fehler | Ursache / Fix |
|---|---|
| `no member named 'toSec' in rclcpp::Time` | `.seconds()` verwenden |
| `cannot convert builtin_interfaces::msg::Time to rclcpp::Time` | `rclcpp::Time(msg->header.stamp)` |
| `can't subtract times with different time sources` (Laufzeit) | `node_->now()` (ROS_TIME) nicht mit `rclcpp::Clock(RCL_STEADY_TIME)` mischen → Laufzeitmessung konsequent über `std::chrono` |
| `no matching function for call to 'toMsg(Eigen::Affine3d&)'` | `tf2_eigen/tf2_eigen.hpp` fehlt oder `tf2_eigen` nicht in CMake |
| `boost::shared_ptr` ≠ `std::shared_ptr` bei PCL-`Ptr` | alle `boost::shared_ptr` im Paket auf `std::shared_ptr` |
| `undefined reference to pcl::...` | `${PCL_LIBRARIES}` an die Library linken, nicht nur an die Executable |
| `create_publisher: topic name must not contain '~' unless at start` | `"~/"` nur als Präfix, keine doppelten `/` |
| `program_options: unrecognised option '--ros-args'` | `rclcpp::remove_ros_arguments` vor `ReadOptions` |

---

## 13. Bekannte Stolpersteine und Unterschiede

1. **Humble statt Jazzy (falls doch 22.04):** `cv_bridge/cv_bridge.h` statt `.hpp`, `tf2_eigen/tf2_eigen.hpp` existiert ab Humble ebenfalls; `rosbag2_cpp::Reader` identisch; MCAP-Plugin unter Humble separat installieren (`ros-humble-rosbag2-storage-mcap`); PCL 1.12, Ceres 2.0 (dort `ceres/manifold.h` vorhanden). Sonst identisch.
2. **Kein Spin im Offline-Modus:** Subscriber-Callbacks (`lodestar::Callback` via Topic, `OdometryKeyframeFuser::pointcloudCallback(PointCloud2)`, `EvalTrajectory::CallbackEst`) werden nie ausgelöst. Für einen späteren Online-Modus: `rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node); exec.spin();` in `main` und `disable_callback=false`.
3. **Zeitquellen:** Header-Stamps kommen aus dem Bag (`RCL_ROS_TIME`); `node_->now()` liefert Wallclock, solange `use_sim_time` false ist. Für konsistente Stamps in RViz2 den Bag-Stamp (`t`) verwenden, wie es `FormatOdomMsg` bereits tut.
4. **`cv::linearPolar`** ist in OpenCV ≥ 4.x deprecated; Ersatz wäre `cv::warpPolar(src, dst, size, center, maxRadius, cv::WARP_FILL_OUTLIERS | cv::WARP_POLAR_LINEAR)`. Nicht zwingend, aber sinnvoll für Zukunftssicherheit.
5. **Hart kodierter Skalierungsfaktor `22.73`** in `EvalTrajectory::Write` (Pixel→Meter) sowie die Kommentare `20.35` (Seadronix) / `16.16` (Pohang) sind dataset-spezifisch. Empfehlung nach der Migration: als CLI-Option `--trajectory_scale` herausziehen.
6. **`exit(0)` in Callbacks** (`lodestar::Callback`, `EvalTrajectory::Save`, `processFrame` bei Registrierungsfehler) umgeht `rclcpp::shutdown()`. Funktioniert, aber sauberer: `rclcpp::shutdown(); std::exit(...)`.
7. **OpenMP + `-O3`:** Über `OpenMP::OpenMP_CXX` und `Release` abgedeckt; die alten manuellen Flags nicht wieder einfügen.
8. **`assert()` im Release-Build** ist deaktiviert – wie unter ROS 1; kein Migrationsproblem.
9. **QoS-Mismatch mit RViz2:** Nur mit Reliable-Publishern arbeiten (siehe 8.3).
10. **`rosbag2` Deserialisierung** ist deutlich langsamer als `rosbag::View::instantiate`; bei großen Bildern (2048×2048 mono8) trotzdem im Bereich weniger ms pro Frame – unkritisch gegenüber der Kreuzkorrelation (3600×3600 Schleife).

---

## 14. Abschluss-Checkliste

- [ ] Ubuntu 24.04 + ROS 2 Jazzy installiert, alle apt-Pakete aus Abschnitt 4
- [ ] `package.xml` format 3, `ament_cmake`, keine ROS-1-Pakete mehr
- [ ] `CMakeLists.txt` ament_cmake, C++17, Install-Regeln für `launch/`, `rviz/`, `scripts/`
- [ ] `statistics`, `utils`: Includes/`ToMs` migriert
- [ ] `pointnormal`: statischer Node + Publisher-Map, `std::shared_ptr`, `msg::`-Typen
- [ ] `registration`, `n_scan_normal`: Node-Parameter durchgereicht, `std::shared_ptr<ceres::Problem>`
- [ ] `lodestar`: Publisher/Subscriber rclcpp, `pcl::toROSMsg`, `tf2::toMsg`, `std::chrono`
- [ ] `odometrykeyframefuser`: `tf2_ros::TransformBroadcaster`, `tf2::eigenToTransform`, `~/`-Topics
- [ ] `eval_trajectory`: `std::filesystem`, `tf2::fromMsg/toMsg`, `rclcpp::Time` in `poseStamped`
- [ ] `lodestar_odom.cpp`: `rclcpp::init`, `rosbag2_cpp::Reader` + `StorageFilter` + `Serialization`, `remove_ros_arguments`, `rclcpp::shutdown`
- [ ] `launch/vis.launch.py`, optional `lodestar_odom.launch.py`
- [ ] `scripts/run_lodestar_odom.sh` mit `ros2 launch`/`ros2 run`
- [ ] `rviz/odom.rviz` im RViz2-Format, Topics `/lodestar_odom_node/radar_registered` und `/lodestar_odom_node/radar_odom`
- [ ] Bags mit `rosbags-convert` konvertiert, `ros2 bag info` zeigt `/radar_image_inrange` als `sensor_msgs/msg/Image`
- [ ] `colcon build` ohne Fehler
- [ ] Vollständiger Lauf erzeugt `eval/01.txt` und `pars.txt`
- [ ] `evo_ape` gegen ROS-1-Referenz ≈ 0
- [ ] README aktualisiert
- [ ] Alte Dateien `launch/vis.launch`, `launch/run_lodestar_odom` gelöscht
