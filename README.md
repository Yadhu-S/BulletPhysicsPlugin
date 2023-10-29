# BulletPhysicsPlugin
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
*Note: Debugging does not work.*

#### Manual compilation
1. Clone [bullet](https://github.com/bulletphysics/bullet3)
2. Compile, `cmake \
    -DLIBRARY_OUTPUT_PATH=<PATH TO THIS REPO>/BulletPhysicsPlugin/Source/ThirdParty/BulletPhysicsEngineLibrary/lib/linux/Debug\
    -DBUILD_SHARED_LIBS=1 \
    -DINSTALL_LIBS=0 \
    -DINSTALL_EXTRA_LIBS=0 \
    -DCMAKE_BUILD_TYPE=Debug \
    .`
	Replace build type as required.
3. `make` , had to use `make -j20` because, compile was not using all cores for some reason.
4. `sudo make install`
5. Link the libs from `/usr/lib` to the `ThirdParty` folder


### Credits
Read this [blog](https://www.stevestreeting.com/2020/07/26/using-bullet-for-physics-in-ue4/). The code in this plugin
is from there.

Originally Forked From: https://github.com/SacredCodeWriter/BulletPhysicsPlugin

