# BulletPhysicsPlugin
Bullet Physics SDK for Unreal Engine 4.26+

### Windows
Clone this repo into your `Engine/Plugin` directory, recompile.

### Linux
1. Clone [bullet](https://github.com/bulletphysics/bullet3)
2. Compile, `cmake -DBUILD_SHARED_LIBS=1 -DINSTALL_LIBS=1 \ -DINSTALL_EXTRA_LIBS=1 \ -DCMAKE_BUILD_TYPE=Release .`
3. `make` , had to use `make -j20` because, compile was not using all cores for some reason.
4. `sudo make install`
5. Link the libs from `/usr/lib` to the `ThirdParty` folder

### Source
Read this [blog](https://www.stevestreeting.com/2020/07/26/using-bullet-for-physics-in-ue4/). The code in this plugin
is from there.

Originally Forked From: https://github.com/SacredCodeWriter/BulletPhysicsPlugin

