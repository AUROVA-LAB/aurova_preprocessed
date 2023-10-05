## Description

The kitti_preprocess project description

# ROS Interface
### Topic publishers
  - ~**odometry_gps** (nav_msgs/Odometry.msg)
  - ~**odom** (nav_msgs/Odometry.msg)
  - /**tf** (tf/tfMessage)
### Topic subscribers
  - ~**fix** (sensor_msgs/NavSatFix.msg)
  - /**tf** (tf/tfMessage)
  - ~**pointcloud** (sensor_msgs/PointCloud2.msg)

### Parameters
- ~**rate** (Double; default: 10.0; min: 0.1; max: 1000) The main node thread loop rate in Hz. 

## Installation

Move to the active workspace:
```bash
roscd && cd ../src
```
Clone the repository: 
```bash
git clone <url>
```
Install ROS dependencies:
```
roscd
cd ..
rosdep install -i -r --from-paths src
```
Compile the workspace:
```
catkin_make
```

## How to use it

- Standalone test

  `roslaunch kitti_preprocess test.launch`

## Disclaimer  

Copyright (C) Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Mantainer IRI labrobotics (labrobotica@iri.upc.edu)

This package is distributed in the hope that it will be useful, but without any warranty. It is provided "as is" without warranty of any kind, either expressed or implied, including, but not limited to, the implied warranties of merchantability and fitness for a particular purpose. The entire risk as to the quality and performance of the program is with you. should the program prove defective, the GMR group does not assume the cost of any necessary servicing, repair  or correction.

In no event unless required by applicable law the author will be liable to you for damages, including any general, special, incidental or consequential damages arising out of the use or inability to use the program (including but not limited to loss of data or data being rendered inaccurate or losses sustained by you or third parties or a failure of the program to operate with any other programs), even if the author has been advised of the possibility of such damages.

You should have received a copy of the GNU Lesser General Public License along with this program. If not, see <http://www.gnu.org/licenses/>