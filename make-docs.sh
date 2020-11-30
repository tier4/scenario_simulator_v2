#!/bin/sh

here=$(cd "$(dirname "$0")" || exit; pwd)

for each in $(find "$here/docs" -name 'Doxyfile')
do
  cd "$(dirname "$each")" || exit
  rm -rf output markdown
  mkdir -p ./markdown/Classes
  doxygen
  doxybook2 --input ./build --output ./markdown -c config.json
done

cd "$here" || exit

mkdocs serve
