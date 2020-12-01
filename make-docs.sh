#!/bin/sh

here=$(cd "$(dirname "$0")" || exit; pwd)

# pip3 install fontawesome_markdown \
#              markdown \
#              mdx_unimoji \
#              mkdocs \
#              mkdocs-material \
#              plantuml-markdown
#              pymdown-extensions \
#              python-markdown-math
#
# mkdir -p "$here/external/doxybook2"
# cd "$here/external/doxybook2" || exit
#
# wget https://github.com/matusnovak/doxybook2/releases/download/v1.2.3/doxybook2-linux-amd64-v1.2.3.zip
#
# unzip doxybook2-linux-amd64-v1.2.3.zip

for each in $(find "$here/docs" -name 'Doxyfile')
do
  cd "$(dirname "$each")" || exit
  rm -rf output markdown
  mkdir -p ./markdown/Classes
  doxygen
  "$here/external/doxybook2/bin/doxybook2" --input ./build --output ./markdown -c config.json
done

cd "$here" || exit

mkdocs serve
