output=compile_commands.json
rm compile_commands.json 2> /dev/null
echo -n '[' >> $output
for file in include/* src/*; do 
cat << EOF >> $output
{
  "directory": "$PWD/build",
  "command": "/usr/bin/clang++ -I/usr/local/include/librealsense -I/usr/local/include/tinysplinecpp.h -I$PWD/./include -isystem /usr/include/opencv -std=c++17",
  "file": "$PWD/$file"
},
EOF
done
echo -n ']' >> $output
