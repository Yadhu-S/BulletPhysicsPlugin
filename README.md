# BulletPhysicsPlugin for Unreal Engine
Bullet Physics SDK for Unreal Engine 4.26+

## Screenshots
| Physics asset | Editor |
|:-------------:|:------:|
| ![alt text](https://github.com/Yadhu-S/BulletPhysicsPlugin/blob/dev/screenshots/UE_physics_view.png?raw=true) | ![alt text](https://github.com/Yadhu-S/BulletPhysicsPlugin/blob/dev/screenshots/UE_viewport.png?raw=true) |



## Status
Implementing and adding support for a custom USkeletalMeshComponent. It supports compound shapes.
*This is a work in progress (very WIP) :), use the originally forked repo mentioned below if you want something usable.*

### Windows
Clone this repo into your `Engine/Plugin` directory, recompile.

### Linux
Clone this repo into your `Engine/Plugin` directory, recompile.
*Note: Debugging does not work. Also, libs are not provided, you need to compile it youself as mentioned below*

#### Manual compilation(bullet 3)
1. Clone [bullet](https://github.com/bulletphysics/bullet3)
2. Compile, `cmake \
    -DLIBRARY_OUTPUT_PATH=<PATH TO THIS REPO>/BulletPhysicsPlugin/Source/ThirdParty/BulletPhysicsEngineLibrary/lib/linux/<RelFolder>\
    -DBUILD_SHARED_LIBS=1 \
    -DINSTALL_LIBS=0 \
    -DINSTALL_EXTRA_LIBS=0 \
    -DCMAKE_BUILD_TYPE=<RelType> \
    .`
	Replace build type as required: `Debug`,`RelWithDebugInfo`,`Release`.
3. `make` , had to use `make -j` because, compile was not using all cores for some reason.


### Credits
Bullet physics(*obv*) [bullet](https://github.com/bulletphysics/bullet3)
Read this [blog](https://www.stevestreeting.com/2020/07/26/using-bullet-for-physics-in-ue4/). The code in this plugin
is from there.

Originally Forked From: https://github.com/SacredCodeWriter/BulletPhysicsPlugin

