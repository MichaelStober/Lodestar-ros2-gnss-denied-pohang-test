# Nächste Schritte: vom migrierten Code zum laufenden System

Zielumgebung: **WSL2 mit Ubuntu 24.04, ROS 2 Jazzy**. Keine ROS-1-Umgebung mehr verfügbar.

Der Code ist portiert und liegt im Repository. Was hier steht, ist der Weg von dort bis zu einer verifizierten, laufenden Odometrie. Die Schritte 1 bis 6 sind Pflicht und bauen aufeinander auf; Schritt 7 ist die Abnahme; Schritt 8 listet Themen, die bewusst offen geblieben sind.

Eine ehrliche Vorbemerkung zum Prüfstand: Der Code wurde gegen die echten Zielbibliotheken (PCL 1.14.0, OpenCV 4.6.0, Ceres 2.2.0, Eigen 3.4.0) kompiliert, gelinkt und über eine synthetische Radarsequenz laufen gelassen, mit sauberem Valgrind-Lauf. Die ROS-Middleware-Ebene lief dabei jedoch gegen nachgebaute Header, weil `packages.ros.org` aus der Build-Umgebung nicht erreichbar war. Signatur- und Konsistenzfehler über alle Dateien hinweg sind damit ausgeschlossen, aber der erste echte `colcon build` kann noch Überladungsauflösungen finden, die nur die originalen rclcpp-Header kennen. Schritt 4 enthält deshalb eine Fehlertabelle mit den Stellen, an denen das am ehesten passiert.

---

## Inhalt

1. WSL2 und Ubuntu 24.04 einrichten
2. ROS 2 Jazzy installieren
3. Repository nach WSL holen und aufräumen
4. Bauen
5. Bag-Dateien konvertieren
6. Erster Lauf und Smoke-Tests
7. Verifikation ohne ROS-1-Referenz
8. Bewusst offene Punkte im Code
9. Kurzreferenz für den Alltag

---

## 1. WSL2 und Ubuntu 24.04 einrichten

Falls WSL2 schon läuft, spring zu 1.3.

### 1.1 Installation

In einer **PowerShell als Administrator**:

```powershell
wsl --install -d Ubuntu-24.04
```

Danach Neustart, dann startet Ubuntu automatisch und fragt nach Benutzername und Passwort.

### 1.2 Prüfen, dass es wirklich WSL**2** ist

```powershell
wsl --list --verbose
```

In der Spalte `VERSION` muss `2` stehen. Falls dort `1` steht:

```powershell
wsl --set-version Ubuntu-24.04 2
wsl --set-default-version 2
```

WSL1 funktioniert für dieses Projekt nicht brauchbar: kein vollständiger Linux-Kernel, deutlich langsamere Dateizugriffe, und RViz2 läuft nicht.

Ebenfalls prüfen:

```powershell
wsl --version
```

Ist der Befehl unbekannt, läuft eine alte, in Windows eingebaute WSL-Version ohne WSLg. Dann einmal `wsl --update` ausführen.

### 1.3 Grafik für RViz2

Unter **Windows 11** ist WSLg eingebaut, RViz2 öffnet sich einfach als Fenster. Test nach der ROS-Installation in Schritt 2:

```bash
rviz2
```

Unter **Windows 10** brauchst du entweder ein aktuelles WSL-Update (`wsl --update`, WSLg wird seit einiger Zeit auch dort ausgeliefert) oder einen X-Server wie VcXsrv mit `export DISPLAY=<Windows-IP>:0`.

Wenn RViz2 startet, aber schwarz bleibt oder mit OpenGL-Fehlern abstürzt, erzwinge Software-Rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

Das kostet Bildrate, ist für das Betrachten von Punktwolken aber völlig ausreichend. Zum Dauerhaftmachen in `~/.bashrc` eintragen.

### 1.4 Arbeitsspeicher konfigurieren (empfohlen)

Der Build ist template-lastig (Ceres, PCL, Eigen). Standardmäßig nimmt WSL2 sich bis zu 50 % des RAM, aber begrenzt die Swap-Größe. Lege unter Windows `C:\Users\michi\.wslconfig` an:

```ini
[wsl2]
memory=12GB
processors=8
swap=8GB
```

Werte an deine Maschine anpassen. Danach `wsl --shutdown` in PowerShell, dann WSL neu starten. Ohne ausreichend RAM bricht `colcon build` mit `c++: fatal error: Killed signal terminated program cc1plus` ab — das ist der OOM-Killer, kein Codefehler.

---

## 2. ROS 2 Jazzy installieren

In der **Ubuntu-Shell** (WSL). Die Paketquelle wird seit 2025 nicht mehr über einen manuell heruntergeladenen Schlüssel eingebunden, sondern über ein `ros2-apt-source`-Paket — ältere Anleitungen im Netz sind an dieser Stelle veraltet.

### 2.1 Paketquelle einbinden

```bash
sudo apt update && sudo apt install -y curl software-properties-common
sudo add-apt-repository -y universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade -y
```

Kontrolle, dass die Quelle greift:

```bash
apt-cache policy ros-jazzy-rclcpp
```

Es muss eine Kandidatenversion erscheinen. Kommt nichts, ist die Paketquelle nicht eingebunden — nicht weitermachen, sonst schlagen alle folgenden Schritte mit irreführenden Fehlern fehl.

### 2.2 ROS 2 und die Paketabhängigkeiten

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools

sudo apt install -y \
  ros-jazzy-rosbag2-cpp ros-jazzy-rosbag2-storage \
  ros-jazzy-rosbag2-storage-mcap ros-jazzy-rosbag2-storage-default-plugins \
  ros-jazzy-cv-bridge ros-jazzy-pcl-conversions \
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-angles ros-jazzy-rviz2 ros-jazzy-tf2-tools
```

`ros-jazzy-desktop` bringt RViz2 und die Standard-Nachrichtenpakete bereits mit; die zweite Zeile ergänzt, was dieses Paket zusätzlich braucht.

### 2.3 Systembibliotheken

```bash
sudo apt install -y libpcl-dev libopencv-dev libceres-dev libeigen3-dev \
  libboost-program-options-dev libomp-dev python3-numpy
```

Versionen gegenprüfen — das sind exakt die, gegen die der Code verifiziert wurde:

```bash
pkg-config --modversion pcl_common    # erwartet 1.14.x
pkg-config --modversion opencv4       # erwartet 4.6.x
pkg-config --modversion eigen3        # erwartet 3.4.x
dpkg -s libceres-dev | grep ^Version  # erwartet 2.2.x
```

### 2.4 rosdep initialisieren

```bash
sudo rosdep init      # schlägt fehl, wenn schon initialisiert - unkritisch
rosdep update
```

### 2.5 Umgebung dauerhaft laden

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 --help    # muss Hilfe ausgeben
```

---

## 3. Repository nach WSL holen und aufräumen

### 3.1 Wichtig: nicht auf `/mnt/c` bauen

Das Repo liegt aktuell unter `C:\Users\michi\Documents\GitHub\Lodestar-ros2-gnss-denied-pohang-test`, aus WSL erreichbar als `/mnt/c/Users/michi/...`. **Dort nicht bauen.** Der Zugriff auf das Windows-Dateisystem läuft über 9p und ist bei den vielen kleinen Dateien eines C++-Builds um ein Vielfaches langsamer; Buildzeiten von 20 Minuten statt 3 sind normal, und `colcon` legt zusätzlich Symlinks an, die auf NTFS Probleme machen.

Arbeite im Linux-Dateisystem unter `~`.

### 3.2 Zuerst die Windows-Seite committen

Die migrierten Dateien liegen im Windows-Arbeitsverzeichnis, sind aber noch nicht committet. In Git Bash oder PowerShell unter Windows:

```bash
cd /c/Users/michi/Documents/GitHub/Lodestar-ros2-gnss-denied-pohang-test
git status
```

Dann die beiden veralteten ROS-1-Startdateien entfernen — das ist der Punkt, den ich nicht für dich erledigen konnte, weil der Dateizugriff auf deinen Rechner seit dem Windows-Update vom 8. September nur schreibend funktioniert:

```bash
git rm launch/vis.launch launch/run_lodestar_odom
```

Beide sind durch `launch/vis.launch.py` beziehungsweise `scripts/run_lodestar_odom.sh` ersetzt. Bleiben sie liegen, stören sie den Build nicht, aber `ros2 launch lodestar_odometry vis.launch` würde später auf eine Datei zeigen, die ROS 2 nicht interpretieren kann.

Dann alles committen:

```bash
git add -A
git status          # kurz gegenlesen, was reingeht
git commit -m "Port to ROS 2 Jazzy (ament_cmake, rclcpp, rosbag2, tf2)"
git push origin main
```

### 3.3 In WSL klonen

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MichaelStober/Lodestar-ros2-gnss-denied-pohang-test.git lodestar_odometry
```

Der Zielordnername ist frei wählbar; `colcon` richtet sich nach `<name>` in `package.xml`, nicht nach dem Ordner. `lodestar_odometry` zu nehmen erspart aber Verwirrung.

Falls du nicht pushen willst, geht auch ein lokaler Klon über die Windows-Kopie — klonen ist in Ordnung, nur das *Bauen* dort ist das Problem:

```bash
git clone /mnt/c/Users/michi/Documents/GitHub/Lodestar-ros2-gnss-denied-pohang-test \
          ~/ros2_ws/src/lodestar_odometry
```

### 3.4 Git-Zeilenenden

Die Originaldateien hatten CRLF-Zeilenenden, die migrierten Dateien haben LF. Damit Git dir das nicht als Komplettänderung jeder Datei anzeigt, unter Windows einmalig:

```bash
git config --global core.autocrlf true
```

In WSL dagegen `input` verwenden:

```bash
git config --global core.autocrlf input
```

---

## 4. Bauen

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lodestar_odometry --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Der erste Build dauert je nach Maschine 3 bis 10 Minuten. Danach:

```bash
source ~/ros2_ws/install/setup.bash
```

Diese Zeile gehört ebenfalls in `~/.bashrc`, **nach** der Jazzy-Zeile.

### 4.1 Erwartete Warnungen

Diese sind harmlos und stammen aus dem Originalcode:

- `unused variable 'bins'`, `'rot_diff'`, `'min_width'`, `'cov'`, `'tar_mean_world'`, `'src_mean_world'` — tote Variablen im Upstream-Code, bewusst nicht entfernt.
- `'cv::linearPolar' is deprecated` — funktioniert in OpenCV 4.6 weiterhin.
- Deprecation-Hinweise zu `ament_target_dependencies` können ab Jazzy auftauchen; das Makro funktioniert in Jazzy noch, entfällt erst später.

### 4.2 Wenn der Build bricht

Der Code wurde gegen nachgebaute rclcpp-Header verifiziert. Die folgenden Stellen sind die realistischen Kandidaten dafür, dass die echten Header strenger sind — mit jeweils der Datei und der Korrektur:

| Fehlermeldung | Datei | Ursache und Behebung |
|---|---|---|
| `no matching function ... create_subscription` mit Lambda | `odometrykeyframefuser.cpp` (Konstruktor) | Das Lambda für die überladene `pointcloudCallback` muss die exakte Signatur treffen. Falls rclcpp meckert, ersetze das Lambda durch `std::bind` mit explizitem Cast: `std::bind(static_cast<void (OdometryKeyframeFuser::*)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)>(&OdometryKeyframeFuser::pointcloudCallback), this, std::placeholders::_1)` |
| `invalid conversion ... ConstSharedPtr` | alle Callbacks | Jazzy erwartet `ConstSharedPtr` **by value** statt `const &`. Dann in Header und Quelle das `&` entfernen. |
| `'toMsg' is not a member of 'tf2'` | `lodestar.cpp`, `odometrykeyframefuser.cpp`, `eval_trajectory.cpp` | `tf2_eigen/tf2_eigen.hpp` fehlt im Include oder `tf2_eigen` nicht in `ament_target_dependencies`. Beides ist gesetzt — falls es dennoch auftritt, zusätzlich `#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>`. |
| `cv_bridge/cv_bridge.hpp: No such file` | `lodestar.h` | Nur auf Humble/älter: dort heißt der Header `.h`. Auf Jazzy ist `.hpp` korrekt. |
| `no member named 'toSec'` / `'toNSec'` | irgendwo | Eine `ros::Time`-Stelle wurde übersehen. `.seconds()` bzw. `.nanoseconds()` verwenden. |
| `can't subtract times with different time sources` **zur Laufzeit** | Zeitmessung | Eine `node_->now()`-Differenz wurde mit einer Bag-Zeit gemischt. Alle Laufzeitmessungen laufen über `std::chrono` und `ToMsSince()`; dort nachsehen. |
| `undefined reference to pcl::...` | Linkphase | `${PCL_LIBRARIES}` muss an die **Bibliothek** gelinkt sein, nicht nur ans Executable. Steht so im CMakeLists — prüfen, ob `find_package(PCL ...)` die Komponenten gefunden hat. |
| `c++: fatal error: Killed signal terminated program cc1plus` | Build bricht scheinbar zufällig ab | Kein Codefehler, sondern RAM. Siehe 1.4, oder `colcon build --executor sequential --parallel-workers 1`. |

Bei einem Fehler, der hier nicht steht: die vollständige Meldung mit Datei und Zeile ist aussagekräftig, der erste Fehler zählt, nicht die Folgefehler.

---

## 5. Bag-Dateien konvertieren

rosbag2 kann ROS-1-`.bag`-Dateien nicht lesen. Die Konvertierung braucht **keine** ROS-1-Installation.

### 5.1 Werkzeug installieren

Ubuntu 24.04 hat ein PEP-668-verwaltetes Python: `pip install --user` wird mit `externally-managed-environment` abgelehnt. Nimm `pipx`:

```bash
sudo apt install -y pipx
pipx ensurepath
exec $SHELL          # damit ~/.local/bin im PATH landet
pipx install rosbags
rosbags-convert --help
```

### 5.2 Bags herüberholen

Liegen die Bags unter Windows, kopiere sie ins Linux-Dateisystem — sequenzielles Lesen über `/mnt/c` geht zwar, ist aber spürbar langsamer:

```bash
mkdir -p ~/ros2_data/odom_test
cp /mnt/c/Users/michi/<Pfad>/test_sequence3.bag ~/ros1_bags/
```

### 5.3 Konvertieren

```bash
rosbags-convert --src ~/ros1_bags/test_sequence3.bag \
                --dst ~/ros2_data/odom_test/test_sequence3
```

Nur das Radar-Topic mitnehmen (deutlich kleiner, schneller):

```bash
rosbags-convert --src ~/ros1_bags/test_sequence3.bag \
                --dst ~/ros2_data/odom_test/test_sequence3 \
                --include-topic /radar_image_inrange
```

MCAP statt SQLite (Jazzy-Standardformat, schnelleres Lesen):

```bash
rosbags-convert --src ... --dst ... --dst-storage mcap
```

Das Ergebnis ist ein **Verzeichnis** mit `metadata.yaml`. Dieser Verzeichnispfad ist das, was `--bag_path` erwartet — nicht die Datei darin.

### 5.4 Kontrolle

```bash
ros2 bag info ~/ros2_data/odom_test/test_sequence3
```

Erwartet: Topic `/radar_image_inrange`, Typ `sensor_msgs/msg/Image`, eine Anzahl N. **Notiere dir N** — das ist die erwartete Posenzahl in Schritt 7.

Encoding prüfen, denn der Code erwartet `mono8`:

```bash
# Terminal 1
ros2 bag play ~/ros2_data/odom_test/test_sequence3
# Terminal 2
ros2 topic echo --once /radar_image_inrange --field encoding
```

Kommt etwas anderes als `mono8` zurück, schlägt `cv_bridge::toCvCopy` zur Laufzeit fehl. Dann muss die Konvertierung oder der Encoding-Parameter angepasst werden.

---

## 6. Erster Lauf und Smoke-Tests

### 6.1 Startet das Executable überhaupt

```bash
ros2 run lodestar_odometry lodestar_odom --help
```

Muss die Optionsliste von boost::program_options ausgeben. Tut es das nicht, stimmt etwas mit der Installation nicht (Schritt 4).

### 6.2 Vollständiger Lauf

```bash
mkdir -p ~/ros2_data/odom_test/test_sequence3_eval

ros2 run lodestar_odometry lodestar_odom \
  --bag_path      ~/ros2_data/odom_test/test_sequence3 \
  --est_directory ~/ros2_data/odom_test/test_sequence3_eval/ \
  --sequence      test_sequence3 \
  --radar_topic   /radar_image_inrange \
  --range-res 0.05 --cost_type P2P --submap_scan_size 1 \
  --registered_min_keyframe_dist 1.5 --res 3 --z-min 60 \
  --weight_option 4 --weight_intensity true \
  --soft_constraint false --disable_compensate true \
  --dataset marine --contour_threshold 214 --k_nearest 20 --job_nr 1
```

Der **abschließende Schrägstrich** bei `--est_directory` ist Pflicht: der Parameter-Dump geht nach `<est_directory>../pars.txt`, und ohne Schrägstrich landet er eine Ebene zu hoch.

Erwartete Ausgabe pro Frame: `Rotation Shifts`, drei Zeitmessungen, `Frame: N, Odom duration: ...`. Am Ende `Distance traveled: ...` und `Trajectoy saved`.

### 6.3 Topics zur Laufzeit prüfen

Während der Lauf läuft, in einem zweiten Terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 node list      # erwartet: /lodestar_odom_node
ros2 topic list
ros2 topic hz /lodestar_odom_node/radar_odom
ros2 run tf2_tools view_frames     # erzeugt frames.pdf: world -> radar_link
```

Erwartete Topics: `/lodestar_odom_node/radar_odom`, `.../radar_odom_keyframe`, `.../radar_registered`, `.../radar_registered_keyframe` sowie die absoluten `/marine/Filtered`, `/radar_imported`, `/rot_lodestar`, `/current_normals`.

Wenn `/lodestar_odom_node/radar_odom` fehlt, aber `/radar_odom` existiert, ist die `~/`-Präfixregel gebrochen — dann würde auch die RViz-Konfiguration ins Leere zeigen.

### 6.4 RViz2

```bash
ros2 launch lodestar_odometry vis.launch.py
```

Fixed Frame ist `world`. Erwartet: die registrierte Punktwolke auf `/lodestar_odom_node/radar_registered` und grüne Odometrie-Pfeile auf `/lodestar_odom_node/radar_odom`.

Zeigt RViz2 nichts, obwohl `ros2 topic hz` Daten meldet, liegt es fast immer an QoS: der Node publiziert *reliable*, RViz2 muss in der Display-Eigenschaft `Reliability Policy` ebenfalls auf `Reliable` stehen. Die mitgelieferte Konfiguration setzt das bereits.

---

## 7. Verifikation ohne ROS-1-Referenz

Ohne alten Build gibt es keine Trajektorie zum Gegenrechnen. Das heißt nicht, dass nicht verifiziert werden kann — es verschiebt nur, *wogegen* geprüft wird. Vier Ebenen, von billig nach aussagekräftig.

### 7.1 Strukturprüfung (sofort, kostet nichts)

Dafür liegt `scripts/check_trajectory.py` im Repo:

```bash
python3 ~/ros2_ws/src/lodestar_odometry/scripts/check_trajectory.py \
  ~/ros2_data/odom_test/test_sequence3_eval/01.txt \
  --frames <N aus ros2 bag info> --rate 4.0
```

Geprüft wird, was jede korrekte Odometrieausgabe erfüllen muss, unabhängig von der Middleware: keine NaN oder Inf, orthonormale Rotationsmatrizen mit Determinante +1, Posenzahl gleich Framezahl, ebene Bewegung mit z = 0, stetige Gierwinkel und keine Ausreißersprünge zwischen aufeinanderfolgenden Posen. Harte Verstöße geben Exit-Code 1.

Das Skript ist gegen eingebaute Fehler getestet: manipulierte NaN, kaputte Rotationsmatrix, künstlicher Positionssprung und komplett statische Trajektorie werden alle erkannt.

Schlägt hier etwas fehl, ist es ein echter Portierungsfehler und keine Kalibrierungsfrage.

### 7.2 Visuelle Kohärenz (die schnellste echte Aussage)

In RViz2 den Decay Time des `radar_registered`-Displays hochsetzen, sodass sich die Punktwolken aufsummieren. Bei korrekter Odometrie ergeben die registrierten Scans eine **scharfe, kohärente Karte**: Kaimauern bleiben Linien, Boote bleiben Punkte. Bei kaputter Registrierung verschmiert alles zu einer Wolke oder die Karte rotiert in sich.

Das ist überraschend aussagekräftig — eine falsch portierte Transformationskette sieht man sofort, weil die Karte auseinanderläuft.

### 7.3 Vergleich gegen Datensatz-Ground-Truth (die eigentliche Abnahme)

Der Pohang Canal Dataset enthält RTK-GPS und eine SLAM-basierte Baseline-Trajektorie. Das ist der Ersatz für die fehlende ROS-1-Referenz und für deine Masterarbeit ohnehin die relevantere Größe: nicht "verhält sich der Port wie der alte Code", sondern "wie gut ist die Odometrie".

```bash
pipx install evo

# Ground Truth in KITTI- oder TUM-Format bringen, dann:
evo_ape kitti ground_truth.txt ~/ros2_data/odom_test/test_sequence3_eval/01.txt -va --plot
evo_rpe kitti ground_truth.txt ~/ros2_data/odom_test/test_sequence3_eval/01.txt -va --plot --delta 100 --delta_unit m
```

`-a` richtet die Trajektorien aneinander aus (Umeyama), was nötig ist, weil die Odometrie im Sensorframe startet. `evo_rpe` mit einem Delta von 100 m ist für Odometrie die aussagekräftigere Metrik als der absolute Fehler, weil sie den unvermeidlichen Drift nicht doppelt bestraft.

Einordnung: Das LodeStar-Paper berichtet für seine maritimen Sequenzen Übersetzungsfehler im niedrigen einstelligen Prozentbereich. Liegst du in derselben Größenordnung, arbeitet der Port korrekt. Liegst du um Faktoren daneben, ist die Ursache mit hoher Wahrscheinlichkeit nicht die Migration, sondern einer der beiden Punkte aus Abschnitt 8.1 und 8.2 — Skalierungsfaktor und RCS-Inversion.

### 7.4 Interne Konsistenz (wenn etwas nicht stimmt)

Zum Eingrenzen:

```bash
# Registrierung abschalten: die Trajektorie darf dann nur noch der LodeStar-Rotation folgen
... --cost_type P2P --submap_scan_size 1 ... # und im Code par.disable_registration setzen

# Anderes Kostenmaß: bei grob abweichenden Ergebnissen ist die Assoziation verdächtig
... --cost_type P2L ...

# Größere Submap: stabilisiert, kostet Rechenzeit
... --submap_scan_size 3 ...
```

Zwei Läufe mit identischen Parametern müssen **bitidentische** Ausgaben liefern:

```bash
diff eval_run1/01.txt eval_run2/01.txt && echo "deterministisch"
```

Tun sie das nicht, gibt es eine nichtdeterministische Datenabhängigkeit — bei OpenMP-Parallelisierung ein realistischer Kandidat. Dann `OMP_NUM_THREADS=1` gegenprüfen.

---

## 8. Bewusst offene Punkte im Code

Diese gehören **nicht** zur Migration und wurden deshalb nicht angefasst. Sie betreffen aber direkt, ob deine Pohang-Auswertung sinnvolle Zahlen liefert.

### 8.1 RCS-Inversion für den Pohang-Datensatz

Im Originalskript stand der Kommentar: *"If you want to use radar from pohang canal dataset, you should change source code (RCS Inversion required)"*. Der Pohang-Datensatz kodiert die Radarintensität invertiert gegenüber dem, was `lodestar.cpp` erwartet.

Konkret wirkt sich das auf die Konturextraktion aus:

```cpp
cv::threshold(cv_marine_img->image, img_th, par.contour_threshold, 255, cv::THRESH_TOZERO);
```

Bei invertierter Kodierung schneidet dieser Schwellwert genau das Falsche weg — starke Rückstreuer werden verworfen, Rauschen bleibt. Die Odometrie läuft dann durch, produziert aber Unsinn.

Behebung: vor dem Threshold invertieren, in `lodestar::Callback` in `src/lodestar_odometry/lodestar.cpp`:

```cpp
if (par.dataset == "pohang")
  cv::bitwise_not(cv_marine_img->image, cv_marine_img->image);
```

Sauberer wäre ein eigener Parameter `--invert_rcs`, weil `dataset` schon anderweitig benutzt wird. Prüfe vorher mit `ros2 topic echo` oder einem Bildbetrachter, wie die Intensitäten in deinen Bags tatsächlich verteilt sind — rate das nicht.

### 8.2 Hart kodierter Skalierungsfaktor 22.73

In `EvalTrajectory::Write` (`src/lodestar_odometry/eval_trajectory.cpp`) werden x und y mit `22.73` multipliziert. Im Code stehen daneben zwei auskommentierte Alternativen:

```
(2778m/2048px)/(0.05m/px)*0.75 = 20.346679687   // seadronix
(1654.8/2048px)/0.05           = 16.16015625    // pohang
```

Der aktive Wert `22.73` passt zu **keinem** von beiden. Für Pohang-Daten wäre nach dieser Rechnung `16.16` richtig. Solange der Faktor falsch steht, ist deine Trajektorie um einen konstanten Faktor fehlskaliert — `evo_ape` mit Skalenkorrektur (`-as` statt `-a`) würde das kaschieren, `evo` ohne Korrektur zeigt es als systematischen Fehler.

Empfehlung: den Faktor als CLI-Option herausziehen, statt ihn im Code zu ändern. Das ist eine kleine Änderung an drei Stellen — `EvalTrajectory::Parameters`, `ReadOptions` und die `Write`-Funktion — und erspart dir, für jeden Datensatz neu zu kompilieren.

### 8.3 Verdeckte Variable in `processFrame`

In `src/lodestar_odometry/odometrykeyframefuser.cpp`:

```cpp
bool success = true;
if(!par.disable_registration)
  { [[maybe_unused]] bool success = radar_reg->Register(...); }   // verdeckt die äußere
```

Das Registrierungsergebnis wird verworfen, `success` bleibt immer `true`. Übernommen aus ROS 1, damit der Port dasselbe tut wie das Original. Konsequenz: fehlgeschlagene Registrierungen werden stillschweigend als Keyframes fusioniert.

Wenn du das reparierst — und für eine Abschlussarbeit spricht einiges dafür —, ändert sich das Verhalten: der `exit(0)`-Zweig bei `success == false` wird scharf. Ersetze diesen besser gleich mit durch etwas Sinnvollerem als einem harten Programmabbruch, etwa Frame überspringen und weiterlaufen. Mach das als **eigenen Commit nach** der verifizierten Migration, nicht währenddessen, sonst weißt du bei einer Abweichung nicht mehr, woher sie kommt.

### 8.4 Online-Betrieb

Der Node läuft rein offline: er liest das Bag sequenziell und braucht deshalb keinen Executor. Alle Subscriber sind mit `disable_callback = true` deaktiviert.

Für echten Sensorbetrieb brauchst du in `main`:

```cpp
rclcpp::executors::SingleThreadedExecutor exec;
exec.add_node(node);
exec.spin();
```

und `disable_callback = false` bei den drei Komponenten. Die Callbacks sind alle portiert und vorhanden; es fehlt nur das Spinnen. Für die Thesis-Auswertung brauchst du das vermutlich nicht.

---

## 9. Kurzreferenz für den Alltag

```bash
# Build nach Codeänderung
cd ~/ros2_ws && colcon build --packages-select lodestar_odometry --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash

# Kompletter Durchlauf mit RViz2
ros2 launch lodestar_odometry lodestar_odom.launch.py \
  bag_path:=$HOME/ros2_data/odom_test/test_sequence3 \
  est_directory:=$HOME/ros2_data/odom_test/test_sequence3_eval/ \
  sequence:=test_sequence3

# Ergebnis prüfen
python3 ~/ros2_ws/src/lodestar_odometry/scripts/check_trajectory.py \
  ~/ros2_data/odom_test/test_sequence3_eval/01.txt --frames <N> --rate 4.0

# Gegen Ground Truth
evo_ape kitti gt.txt ~/ros2_data/odom_test/test_sequence3_eval/01.txt -va --plot

# Bag-Info
ros2 bag info ~/ros2_data/odom_test/test_sequence3

# Sauber neu bauen
cd ~/ros2_ws && rm -rf build install log && colcon build --packages-select lodestar_odometry
```

**Reihenfolge der Abnahme:** Schritt 4 grün → Schritt 6.2 läuft durch → 7.1 ohne Fehler → 7.2 Karte ist scharf → 8.1 und 8.2 klären → 7.3 gegen Ground Truth. Erst danach lohnt es, an Parametern zu drehen.

---

## Quellen

- [ROS 2 Jazzy Installation (Debian-Pakete)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Installation, Setup der Paketquelle über ros2-apt-source](https://ros2-tutorial.readthedocs.io/en/latest/installation.html)
- [Pohang Canal Dataset — Sensorik und Baseline-Trajektorie](https://github.com/dhchung/pohang_canal_dataset)
- [Pohang Canal Dataset, IJRR-Veröffentlichung](https://journals.sagepub.com/doi/10.1177/02783649231191145)
- [rosbags — Konvertierung ROS 1 nach ROS 2](https://ternaris.gitlab.io/rosbags/)
- [evo — Trajektorienauswertung](https://github.com/MichaelGrupp/evo)
