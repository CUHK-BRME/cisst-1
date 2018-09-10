#include <chrono>
#include <cisstCommon/cmnRandomSequence.h>
#include <cisstRobot/robManipulator.h>

vctDynamicVector<double> getInitialJointPosition() {
  vctDynamicVector<double> q(6, 0.0);
  q[0] = 0.0;
  q[1] = 0.0;
  q[2] = 0.8;
  q[3] = 0.0;
  q[4] = 0.0;
  q[5] = 0.0;
  return q;
}

vctDynamicVector<double>
addDeltaJointPosition(const vctDynamicVector<double> q) {

  vctDynamicVector<double> qnext(q);
  qnext[0] -= 30 * cmnPI_180;
  qnext[3] += 60 * cmnPI_180;
  qnext[4] -= 60 * cmnPI_180;
  qnext[5] -= 60 * cmnPI_180;
  return qnext;
}

int main(int argc, char **argv) {

  std::ifstream jsonStream;
  jsonStream.open(argv[1]);

  Json::Value config, configJson;
  Json::Reader jsonReader;
  if (!jsonReader.parse(jsonStream, config)) {
    std::cout << "please provide the path of manipulator config json file as "
                 "an argument\n";
    return -1;
  }

  if (config.isNull()) {
    std::cout << "fail to parse json file\n";
    return -1;
  }

  configJson = config["DH"];
  const Json::Value jsonLinks = configJson["links"];

  robManipulator manipulator;
  manipulator.LoadRobot(configJson);

  vctFrame4x4<double> ToolOffsetTransformation;

  const Json::Value jsonToolTip = config["tooltip-offset"];
  if (!jsonToolTip.isNull()) {

    cmnDataJSON<vctFrm4x4>::DeSerializeText(ToolOffsetTransformation,
                                            jsonToolTip);
    robManipulator *ToolOffset = new robManipulator(ToolOffsetTransformation);
    manipulator.Attach(ToolOffset);
  }

  const vctDynamicVector<double> q = getInitialJointPosition();
  vctDynamicVector<double> q_js = q;
  vctDynamicVector<double> q_je = q;
  const vctDynamicVector<double> q_next = addDeltaJointPosition(q);

  std::cout << "using Js, inverse kinematics loop count = ";
  manipulator.InverseKinematics(q_js, manipulator.ForwardKinematics(q_next), true);
  std::cout << "using Je, inverse kinematics loop count = ";
  manipulator.InverseKinematics(q_je, manipulator.ForwardKinematics(q_next), false);

  q_js.ToStream(std::cout << "Using Js, inverse kinematics output q = \n");
  std::cout << '\n';

  q_je.ToStream(std::cout << "Using Je, inverse kinematics output q = \n");
  std::cout << '\n';

  vctDynamicMatrix<double> Js(6,6);
  manipulator.JacobianSpatial(q,Js);
  q.ToStream(std::cout << "For arm configuration q = ");
  Js.ToStream(std::cout << "\nJs = \n");
  std::cout << '\n';
  manipulator.JacobianSpatial(q_js,Js);
  q_js.ToStream(std::cout << "For arm configuration q = ");
  Js.ToStream(std::cout << "\nJs = \n");
  std::cout << '\n';
  return 0;
}
