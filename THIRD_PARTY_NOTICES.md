# Third-party notices / 外部依存のライセンス

## Scope / 対象範囲

The [MIT license](LICENSE) covers this repository's original code and content. It does not relicense ROS, Gazebo, Qt, Python libraries, fonts, or other third-party software. The dependencies below are installed separately through apt as described in [SETUP](docs/SETUP.md); their libraries and upstream source are not vendored in this repository.

[MITライセンス](LICENSE)は、このリポジトリの独自コードとコンテンツに適用します。ROS、Gazebo、Qt、Pythonライブラリ、フォントなどの外部ソフトウェアをMITへ変更するものではありません。以下の依存は[SETUP](docs/SETUP.md)に従ってaptで別途導入し、そのライブラリや上流ソースをリポジトリ内に同梱していません。

This is a guide to the direct dependencies used by the project, checked against Ubuntu 24.04 / ROS 2 Jazzy package notices and upstream sources on 2026-10-05. It is not a complete inventory of Gazebo, ROS, Qt plugins, native image/numerical libraries, or their transitive dependencies. Package revisions may change. Original copyright notices, per-file licenses, and notices for the exact installed or redistributed versions remain authoritative.

これは2026-10-05にUbuntu 24.04 / ROS 2 Jazzyのパッケージ文書と上流ソースで確認した、プロジェクトの直接依存の案内です。Gazebo、ROS、Qtプラグイン、画像・数値計算用のネイティブライブラリや推移的依存をすべて列挙するものではありません。パッケージの更新後や再配布時は、対象バージョンの著作権表示、ファイルごとのライセンス、付属通知を確認してください。

## Direct dependencies / 直接依存

Names in the attribution column identify upstream authors/projects; the linked sources and installed copyright files contain the complete notices.

帰属欄は上流の作者・プロジェクトを示します。完全な著作権表示はリンク先とインストール済みのcopyrightファイルにあります。

| Dependency / 依存 | Observed version / 確認版 | Attribution / 帰属 | License / ライセンス・上流資料 |
| --- | --- | --- | --- |
| `rclpy` | 7.1.11 | Esteve Fernandez, Ivan Paunovic, Jacob Perron, William Woodall; ROS 2 contributors | [Apache-2.0](https://github.com/ros2/rclpy/blob/jazzy/LICENSE) |
| `ros_gz`, `ros_gz_sim`, `ros_gz_bridge` | 1.0.24 | Louise Poubel, Shivesh Khaitan, Carlos Agüero, Addisu Taddese, Alejandro Hernandez; contributors | [Apache-2.0](https://github.com/gazebosim/ros_gz/blob/jazzy/LICENSE) |
| `slam_toolbox` | 2.8.5 | Steve Macenski; contributors | [LGPL-2.1](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/LICENSE); installed manifest labels it `LGPL` |
| `nav2_amcl` | 1.3.13 | Willow Garage, Player/AMCL and Nav2 contributors | [LGPL-2.1-or-later](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/package.xml); [source notices](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp) |
| `nav2_map_server` | 1.3.13 | Nav2 and map_server contributors | [Apache-2.0 and BSD-3-Clause](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_map_server/package.xml), per file |
| `nav2_lifecycle_manager` | 1.3.13 | Intel Corporation, Samsung Research America; Nav2 contributors | [Apache-2.0](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_lifecycle_manager/src/lifecycle_manager.cpp) |
| `robot_localization` | 3.8.3 | Tom Moore, Charles River Analytics, Austin Robot Technology / University of Texas at Austin; contributors | Installed manifest: Apache-2.0; upstream [LICENSE](https://github.com/cra-ros-pkg/robot_localization/blob/3.8.3/LICENSE): BSD notices for the identified code. Preserve both and check individual files. / 両方の表示を保持し、ファイルごとに確認。 |
| NumPy (`python3-numpy`) | 1.26.4 | NumPy Developers; component authors | Main project [BSD-3-Clause](https://github.com/numpy/numpy/blob/v1.26.4/LICENSE.txt); component licenses also apply |
| SciPy (`python3-scipy`) | 1.11.4 | Enthought, Inc.; SciPy Developers; component authors | Main project [BSD-3-Clause](https://github.com/scipy/scipy/blob/v1.11.4/LICENSE.txt); component licenses also apply |
| PyYAML (`python3-yaml`) | 6.0.1 | Ingy döt Net, Kirill Simonov | [MIT](https://github.com/yaml/pyyaml/blob/6.0.1/LICENSE) |
| Pillow (`python3-pil`) | 10.2.0 | Secret Labs AB, Fredrik Lundh, Jeffrey A. Clark (Alex); contributors | [HPND / PIL license](https://github.com/python-pillow/Pillow/blob/10.2.0/LICENSE) |
| psutil (`python3-psutil`) | 5.9.8 | Jay Loden, Dave Daeschler, Giampaolo Rodola | [BSD-3-Clause](https://github.com/giampaolo/psutil/blob/release-5.9.8/LICENSE) |
| PySide2 `QtCore`, `QtGui`, `QtWidgets`, `QtTest`; Shiboken2 runtime | 5.15.13 | The Qt Company Ltd.; PySide and other contributors | Runtime available under [LGPL-3.0](https://code.qt.io/cgit/pyside/pyside-setup.git/plain/LICENSE.LGPLv3?h=5.15); source package also contains files under other licenses |
| Qt 5 Core, Gui, Widgets, Test | 5.15.13 | The Qt Company Ltd.; other copyright holders | Runtime available under [LGPL-3.0](https://code.qt.io/cgit/qt/qtbase.git/plain/LICENSE.LGPL3?h=5.15); third-party components retain their own licenses |
| Noto Sans CJK JP (`fonts-noto-cjk`) | apt package 1:20230817+repack1-3 | Google; Noto contributors | [SIL Open Font License 1.1](https://github.com/notofonts/noto-cjk/blob/main/Sans/LICENSE) for font files; packaging has separate notices |

The ROS entries describe these packages only, not all of ROS 2 or Gazebo. NumPy and SciPy's package copyright files list additional component licenses; their main BSD license does not replace those notices.

ROS欄のライセンスは記載したパッケージの情報であり、ROS 2やGazebo全体を一つのライセンスとみなすものではありません。NumPyとSciPyにも構成要素ごとのライセンスがあり、主プロジェクトのBSDライセンスでそれらを置き換えることはできません。

## Qt / LGPL use and redistribution / Qt・LGPLの利用と再配布

The application imports PySide2 dynamically and uses the system's shared Qt 5 libraries. Runtime GUI modules are `QtCore`, `QtGui`, and `QtWidgets`; validation also uses `QtTest`. This repository supplies Python source and does not statically link, freeze, or bundle Qt/PySide2 binaries. It currently imports no GPL-only Qt modules. The documented runtime uses the LGPL option; adding modules or changing packaging requires checking the resulting licenses again.

アプリケーションはPySide2を動的に読み込み、システムのQt 5共有ライブラリを使用します。GUIは`QtCore`、`QtGui`、`QtWidgets`を使い、検証では`QtTest`も使います。このリポジトリはPythonソースを提供し、Qt/PySide2の静的リンク、実行ファイル化、バイナリ同梱は行いません。現在GPLのみで提供されるQtモジュールは読み込みません。案内する実行環境ではLGPLの選択肢を使用し、モジュール追加や配布形式の変更時には再確認が必要です。

If you redistribute libraries, a container, an appliance, or an application bundle, comply with the licenses of the exact components you distribute. For LGPL components this includes retaining copyright/license notices, supplying the required license texts (LGPLv3 also incorporates GPLv3), and providing corresponding library source, including patches and build material, through a method permitted by the applicable license. Keep a suitable mechanism for users to replace/relink the libraries with compatible modified versions; do not restrict the reverse engineering needed to debug those modifications. Provide installation/relinking information where required. Dynamic imports alone do not satisfy every redistribution obligation. See [LGPLv3 §4](https://www.gnu.org/licenses/lgpl-3.0.html) and [LGPLv2.1 §§4–6](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html).

ライブラリ、コンテナ、機器、アプリケーション一式を再配布する場合は、実際に配布する各構成要素の条件に従ってください。LGPLでは、著作権・ライセンス表示と必要なライセンス全文の保持（LGPLv3はGPLv3も参照）、パッチやビルド資料を含む対応ライブラリソースの適切な提供が必要です。利用者が互換性のある改変版ライブラリへ交換・再リンクできる仕組みを維持し、その改変をデバッグするためのリバースエンジニアリングを制限しないでください。条件に応じてインストール・再リンクに必要な情報も提供します。動的インポートだけですべての再配布条件を満たすわけではありません。[LGPLv3 §4](https://www.gnu.org/licenses/lgpl-3.0.html)、[LGPLv2.1 §§4–6](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html)を参照してください。

## Installed notices, source, and fonts / インストール済み通知・ソース・フォント

On Ubuntu, read `/usr/share/doc/<binary-package>/copyright` for the package you use. ROS package manifests are also available at `/opt/ros/jazzy/share/<ros_package>/package.xml`. In particular:

Ubuntuでは使用するパッケージの`/usr/share/doc/<binary-package>/copyright`を参照してください。ROSのmanifestは`/opt/ros/jazzy/share/<ros_package>/package.xml`にもあります。Qt関連の例は次のとおりです。

- `/usr/share/doc/python3-pyside2.qtcore/copyright` (also `qtgui`, `qtwidgets`, `qttest`)
- `/usr/share/doc/libpyside2-py3-5.15t64/copyright`
- `/usr/share/doc/libshiboken2-py3-5.15t64/copyright`
- `/usr/share/doc/libqt5core5t64/copyright` (also `libqt5gui5t64`, `libqt5widgets5t64`, `libqt5test5t64`)
- `/usr/share/doc/fonts-noto-cjk/copyright`

Qt for Python source is published in the [official source archive](https://download.qt.io/official_releases/QtForPython/pyside2/) and [source repository](https://code.qt.io/cgit/pyside/pyside-setup.git/). Qt 5 base source is in the [Qt source repository](https://code.qt.io/cgit/qt/qtbase.git/). Ubuntu's corresponding source packages include distribution patches: [pyside2 5.15.13-1](https://launchpad.net/ubuntu/+source/pyside2/5.15.13-1) and [qtbase-opensource-src 5.15.13+dfsg-1ubuntu1](https://launchpad.net/ubuntu/+source/qtbase-opensource-src/5.15.13+dfsg-1ubuntu1). With matching source repositories enabled, `apt-get source <source-package>=<source-version>` retrieves that revision. Use `dpkg-query -W -f='${binary:Package} ${Version} ${source:Package} ${source:Version}\n' <binary-package>` to identify it. An upstream URL alone is not a substitute for a distributor's required source provision.

Qt for PythonとQt 5の上流ソースは上記の公式アーカイブ・リポジトリにあります。Ubuntuの対応ソースパッケージには配布元のパッチも含まれます。対象のソースリポジトリを有効にすると、上記の`apt-get source`で対応版を取得できます。`dpkg-query`でバイナリに対応するソース名と版を確認してください。上流URLだけの記載で、再配布者に必要なソース提供を代替できるとは限りません。

Noto Japanese fonts are installed as system dependencies. Locally available Windows fonts such as Meiryo are subject to their own terms; Windows `.ttc` font binaries are excluded from the repository release and are not covered by its MIT license. Font configuration files do not grant redistribution rights to the fonts they reference.

日本語用Notoフォントはシステム依存として導入します。ローカルのMeiryoなどWindowsフォントには固有の利用条件があり、Windowsの`.ttc`フォント本体はリポジトリの配布対象から除外し、MITの対象に含めません。フォント設定ファイルは、参照するフォントの再配布権を付与するものではありません。
