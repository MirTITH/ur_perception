# 机械臂感知抓取

## TODO
- [ ] 修Gazebo
- [ ] 实现代码控制末端

## Note

xacro 引用文件要用以下形式，否则 Gazebo 加载不了
```xml
<mesh filename="file://$(find hdz_simplified)/meshes/hdz_steel_frame_link.STL" />
```

## Done
- [x] 测量得到更精准的撼地者模型