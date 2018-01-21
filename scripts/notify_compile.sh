clear
binary=PowerUp2018
pushd build
inotifywait -e modify -r ../src/ ../include/ -m |
	while read -r x; do
		echo "FILE: $x"
		#make
	done &
	inotify_pid=$!

	while read -r k; do
		case "$k" in
			"q")
				kill $binary_pid 2>/dev/null
				;;
			"qa")
				kill $inotify_pid 2>/dev/null
				kill $binary_pid 2>/dev/null
				pkill -P $$ 2>/dev/null
				exit
				;;
			*)
				kill $binary_pid 2>/dev/null
				./$binary &
				binary_pid=$!
		esac
	done
