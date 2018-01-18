output=compile_commands.json
rm compile_commands.json 2> /dev/null
echo -n '[' >> $output
for file in include/* src/*; do 
cat << EOF >> $output
{
  "directory": "/home/duncan/Projects/Robotics/PowerUp_Jetson_2018/build",
  "command": "/usr/bin/clang++   -I/usr/local/include/librealsense -I/usr/local/include/tinysplinecpp.h -I/home/duncan/Projects/Robotics/PowerUp_Jetson_2018/./include -isystem /usr/include/opencv   -std=c++11    -o CMakeFiles/PowerUp2018.dir/src/field_renderer.cpp.o -c /home/duncan/Projects/Robotics/PowerUp_Jetson_2018/src/field_renderer.cpp",
  "file": "$PWD/$file"
},
EOF
done
echo -n ']' >> $output
