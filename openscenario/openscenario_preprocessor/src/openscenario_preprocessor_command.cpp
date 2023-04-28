// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <boost/program_options.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <nlohmann/json.hpp>
#include <openscenario_preprocessor/openscenario_preprocessor.hpp>
#include <openscenario_preprocessor/t4v2.hpp>
#include <openscenario_preprocessor/tojson.hpp>

const std::string_view template_scenario = R"###(
<OpenSCENARIO>
    <FileHeader author="" date="2022-03-04T18:06:53+09:00" description="" revMajor="0" revMinor="0"/>
    <ParameterValueDistribution>
        <ScenarioFile filepath=""/>
        <Deterministic>
            <DeterministicMultiParameterDistribution>
                <ValueSetDistribution>
                    <ParameterValueSet>
                    </ParameterValueSet>
                </ValueSetDistribution>
            </DeterministicMultiParameterDistribution>
        </Deterministic>
    </ParameterValueDistribution>
</OpenSCENARIO>)###";

auto create_parameter_value_distribution_from_json(
  const boost::filesystem::path & scenario_path, const nlohmann::json & json_parameters)
{
  pugi::xml_document script;
  script.load_string(template_scenario.data());

  script.document_element()
    .select_node(pugi::xpath_query{"/OpenSCENARIO/ParameterValueDistribution/ScenarioFile"})
    .node()
    .attribute("filepath")
    .set_value(scenario_path.c_str());

  auto value_set_node =
    script.document_element()
      .select_node(pugi::xpath_query{
        "/OpenSCENARIO/ParameterValueDistribution/Deterministic/"
        "DeterministicMultiParameterDistribution/ValueSetDistribution/ParameterValueSet"})
      .node();

  for (auto const & json_parameter : json_parameters.items()) {
    auto parameter_assignment_node = value_set_node.append_child("ParameterAssignment");
    parameter_assignment_node.append_attribute("parameterRef") = json_parameter.key().c_str();
    parameter_assignment_node.append_attribute("value") =
      json_parameter.value().get<std::string>().c_str();
  }
  return script;
}

int main(const int argc, char const * const * const argv)
{
  using namespace boost::program_options;

  options_description description("openscenario_preprocessor_command");

  //  -o <directory> -p param.json -s scenario.yaml
  description.add_options()(
    "output-directory,o",
    value<std::string>()->default_value("/tmp/openscenario_preprocessor/derived"),
    "path of output directory")(
    "format,f", value<openscenario_preprocessor::ScenarioFormat>()->multitoken(),
    "output scenario format (t4v2 / xosc)")(
    "parameters,p", value<std::string>()->default_value("null"), "parameters in json format")(
    "scenario,s", value<std::string>(), "path of scenario file")("help,H", "help");

  variables_map vm;
  store(parse_command_line(argc, argv, description), vm);
  notify(vm);

  auto output_directory_option = boost::filesystem::path(vm["output-directory"].as<std::string>());
  auto format_option = vm["format"].as<openscenario_preprocessor::ScenarioFormat>();
  auto parameters_option = boost::filesystem::path(vm["parameters"].as<std::string>());
  auto scenario_option = boost::filesystem::path(vm["scenario"].as<std::string>());

  auto scenario_path = boost::filesystem::path(scenario_option);

  boost::filesystem::path tmp_output_directory = "/tmp/openscenario_preprocessor";
  if(not boost::filesystem::exists(tmp_output_directory)){
    boost::filesystem::create_directories(tmp_output_directory);
  }

  std::vector<boost::filesystem::path> xosc_scenario_paths;

  // preprocess t4v2 format and convert to xosc scenarios
  if (scenario_path.extension() == ".yaml" or scenario_path.extension() == ".yml") {
    openscenario_preprocessor::T4V2 t4v2;
    auto xosc_string_scenarios = t4v2.deriveToXoscStringScenarios(scenario_path);


    boost::filesystem::path t4v2_output_directory = "/tmp/openscenario_preprocessor/t4v2_derived";
    if(not boost::filesystem::exists(t4v2_output_directory)){
      boost::filesystem::create_directories(t4v2_output_directory);
    }
    for (const auto & xosc_string_scenario : xosc_string_scenarios | boost::adaptors::indexed()) {
//      std::cout << xosc_string_scenario.value() << std::endl;
      boost::filesystem::path xosc_scenario_path =
        t4v2_output_directory / (std::to_string(xosc_string_scenario.index()) + ".xosc");
      std::ofstream ofs(xosc_scenario_path.c_str());
      ofs << xosc_string_scenario.value().c_str();
      ofs.close();
      xosc_scenario_paths.push_back(xosc_scenario_path);
    }
//    std::cout << "finish writing xosc scenarios from t4v2 format" << std::endl;
  } else {
    xosc_scenario_paths.push_back(scenario_path);
  }

  // derive for given parameters
  if (parameters_option != "null") {
    for (auto scenario_path : xosc_scenario_paths) {
      auto parameter_value_distribution = create_parameter_value_distribution_from_json(
        scenario_path, nlohmann::json::parse(parameters_option.c_str()));

      parameter_value_distribution.save_file(
        "/tmp/openscenario_preprocessor/parameter_value_distribution.xosc");

      openscenario_preprocessor::Preprocessor preprocessor(output_directory_option);

//        try {
          preprocessor.preprocessScenario(
            "/tmp/openscenario_preprocessor/parameter_value_distribution.xosc", format_option);
//        }catch (rapidxml::parse_error & ex){
//                std::cerr << "[Error] something went wrong during deriving scenario : " << ex.what()
//                        << ", " << ex.where<char>() << std::endl;
////                return 1;
//        }
      //  } catch (std::runtime_error & ex) {
      //    std::cerr << "[Error] something went wrong during deriving scenario : " << ex.what()
      //              << std::endl;
      //    return 1;
      //  }catch (std::exception & ex) {
      //    std::cerr << "[Error] unknown error occured during deriving scenario : " << ex.what()
      //              << std::endl;
      //    std::cerr << "Please contact developers of this software." << std::endl;
      //    return 1;
      //  }
    }
  } else {
    // TODO
  }
  return 0;
}
