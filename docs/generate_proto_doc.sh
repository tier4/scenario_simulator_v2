docker pull pseudomuto/protoc-gen-doc
docker run --rm \
  -v $(pwd)/../simulation/simulation_interface/proto:/protos \
  -v $(pwd)/proto_doc:/out \
  pseudomuto/protoc-gen-doc --doc_opt=markdown,docs.md