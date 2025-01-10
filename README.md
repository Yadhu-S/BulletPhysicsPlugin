# BulletPhysicsPlugin for Unreal Engine
Bullet Physics SDK for Unreal Engine 

## Screenshots
![alt text](https://github.com/Yadhu-S/BulletPhysicsPlugin/blob/68dad0b375008f17c003d37499c1398a1eaa7626/screenshots/UE_viewport.png) 
![alt text](https://github.com/Yadhu-S/BulletPhysicsPlugin/blob/2022f246308845c2990ac2539dfe36abaa5adea7/screenshots/UE_physics_view.png) 



## Status Experimental, but usable.

### Features:
* A custom USkeletalMeshComponent.
* Supports compound shapes.

#### Consider making your own fork if you decide to use this.
~~*This is a work in progress (very WIP) :), use the originally forked repo mentioned below if you want something usable.*~~

## How to use

### Adding objects to the sim
There are multiple ways to add bodies to the jolt sub system. 
Easiest way is to use and `ActorTag`. Tag dynamic Bodies with `B_DYNAMIC` and static bodies with `B_STATIC`

### Ticking
This will be handled automatically by UTickableWorldSubsystem.

### Debug Rendering

To enable debug renderer, call the EnableDebugDrawer as shown in the screenshot


## Compiling
#### Windows
Clone this repo into a `Plugin` directory of your choice, `Engine` or `<Your project Dir>`.

### Linux
Clone this repo into a `Plugin` directory of your choice, `Engine` or `<Your project Dir>`.

#### Mac
Might work, no idea

#### Manual compilation (bullet 3) 
*You don't need to do this. Just putting it here if you need to compile it*
1. Clone [bullet](https://github.com/bulletphysics/bullet3)
2. Compile,

```bash
cmake \
    -DLIBRARY_OUTPUT_PATH=<PATH TO THIS REPO>/BulletPhysicsPlugin/Source/ThirdParty/BulletPhysicsEngineLibrary/lib/linux/<RelFolder>\
    -DBUILD_SHARED_LIBS=1 \
    -DINSTALL_LIBS=0 \
    -DINSTALL_EXTRA_LIBS=0 \
    -DCMAKE_BUILD_TYPE=<RelType> \ # Replace build type as required: `Debug`,`RelWithDebugInfo`,`Release`.
    .
```

3. `make`, had to use `make -j` because, compile was not using all cores for some reason.



### Credits
Bullet physics(*obv*) [bullet](https://github.com/bulletphysics/bullet3)
Read this [blog](https://www.stevestreeting.com/2020/07/26/using-bullet-for-physics-in-ue4/). The code in this plugin
is from there.

Originally Forked From: https://github.com/SacredCodeWriter/BulletPhysicsPlugin

