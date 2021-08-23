%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/noetic/.*$
%global __requires_exclude_from ^/opt/ros/noetic/.*$

Name:           ros-noetic-floam
Version:        0.1.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS floam package

License:        BSD
URL:            https://wanghan.pro
Source0:        %{name}-%{version}.tar.gz

Requires:       ceres-solver-devel
Requires:       eigen3-devel
Requires:       pcl-devel
Requires:       ros-noetic-eigen-conversions
Requires:       ros-noetic-geometry-msgs
Requires:       ros-noetic-hector-trajectory-server
Requires:       ros-noetic-nav-msgs
Requires:       ros-noetic-nodelet
Requires:       ros-noetic-pcl-ros
Requires:       ros-noetic-rosbag
Requires:       ros-noetic-roscpp
Requires:       ros-noetic-rospy
Requires:       ros-noetic-sensor-msgs
Requires:       ros-noetic-std-msgs
Requires:       ros-noetic-tf2
Requires:       ros-noetic-tf2-geometry-msgs
BuildRequires:  ceres-solver-devel
BuildRequires:  eigen3-devel
BuildRequires:  pcl-devel
BuildRequires:  ros-noetic-catkin
BuildRequires:  ros-noetic-eigen-conversions
BuildRequires:  ros-noetic-geometry-msgs
BuildRequires:  ros-noetic-hector-trajectory-server
BuildRequires:  ros-noetic-nav-msgs
BuildRequires:  ros-noetic-nodelet
BuildRequires:  ros-noetic-pcl-ros
BuildRequires:  ros-noetic-rosbag
BuildRequires:  ros-noetic-roscpp
BuildRequires:  ros-noetic-rospy
BuildRequires:  ros-noetic-sensor-msgs
BuildRequires:  ros-noetic-std-msgs
BuildRequires:  ros-noetic-tf2
BuildRequires:  ros-noetic-tf2-geometry-msgs
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
A generic Lidar SLAM package based on FLOAM which was based on ALOAM by HKUST
and LOAM by CMU

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/noetic/setup.sh" ]; then . "/opt/ros/noetic/setup.sh"; fi
mkdir -p obj-%{_target_platform} && cd obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/noetic" \
    -DCMAKE_PREFIX_PATH="/opt/ros/noetic" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
    -DCATKIN_BUILD_BINARY_PACKAGE="1" \
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/noetic/setup.sh" ]; then . "/opt/ros/noetic/setup.sh"; fi
%make_install -C obj-%{_target_platform}

%files
/opt/ros/noetic

%changelog
* Sun Aug 22 2021 Evan Flynn <evanflynn.msu@gmail.com> - 0.1.0-1
- Autogenerated by Bloom

