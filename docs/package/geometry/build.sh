rm -rf output
rm -rf markdown
doxygen Doxyfile
mkdir -p ./markdown/Classes
doxybook2 --input ./build/ --output ./markdown -c config.json
