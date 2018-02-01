grep -e "^#define.*_H" -r src/ include/ | column -t -s ':'
