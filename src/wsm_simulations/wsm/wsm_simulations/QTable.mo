within wsm_simulations;

model QTable
  parameter Integer stateDims = 3 "How many dimensions exist in the state";
  parameter Boolean loadModel = false;
  parameter Boolean saveModel = false;
  parameter String modelURI = "" "File to existing .qtable file to initially load from";
  parameter Modelica.SIunits.Time savingInterval(min = 0) = 60 if saveModel "How often to save";
  parameter Boolean useResetPort = false;
  Modelica.Blocks.Interfaces.RealInput learning_rate annotation(Placement(visible = true, transformation(origin = {-150, 8.366}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-10, -100}, extent = {{-15, -15}, {15, 15}}, rotation = -270)));
  Modelica.Blocks.Interfaces.RealInput epsilon annotation(Placement(visible = true, transformation(origin = {-150, -25}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {35, -100}, extent = {{-15, -15}, {15, 15}}, rotation = -270)));
  Modelica.Blocks.Interfaces.RealInput discount_factor annotation(Placement(visible = true, transformation(origin = {-150, -52.799}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {80, -100}, extent = {{-15, -15}, {15, 15}}, rotation = -270)));
  Modelica.Blocks.Interfaces.RealInput state[stateDims] annotation(Placement(visible = true, transformation(origin = {-17.522, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput reward annotation(Placement(visible = true, transformation(origin = {-150, 38.107}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  SettingConnector stateSettingConnector1[stateDims] annotation(Placement(visible = true, transformation(origin = {-100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -100}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  SettingConnector actionSettingConnector1 annotation(Placement(visible = true, transformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-50, -100}, extent = {{-11.242, -11.242}, {11.242, 11.242}}, rotation = 0)));
  discrete Modelica.Blocks.Interfaces.RealOutput action annotation(Placement(visible = true, transformation(origin = {45, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {105, 20}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanInput trigger annotation(Placement(visible = true, transformation(origin = {-150, 63.458}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanInput reset if useResetPort "If simulation resets and the qtable shouldn't update from the past time-step." annotation(Placement(visible = true, transformation(origin = {-142.8, 56.258}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-90, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  QTableOBJ qtable = QTableOBJ.constructor(if not loadModel then "" else Modelica.Utilities.Files.loadResource(modelURI));

  class QTableOBJ
    extends ExternalObject;

    function constructor
      input String filepath;
      output QTableOBJ qtable;
    
      external "C" qtable = QTableConstructor(filepath) annotation(Include = "#include \"WSM_QTable.h\"", Library = "wsm_simulations_QTable", IncludeDirectory = "modelica://wsm_simulations/../../include/wsm_simulations", LibraryDirectory = "modelica://wsm_simulations/../../../../install/lib/");
    end constructor;

    function destructor
      input QTableOBJ qtable;
    
      external "C" QTableDestructor(qtable) annotation(Include = "#include \"WSM_QTable.h\"", Library = "wsm_simulations_QTable", IncludeDirectory = "modelica://wsm_simulations/../../include/wsm_simulations", LibraryDirectory = "modelica://wsm_simulations/../../../../install/lib/");
    end destructor;
  end QTableOBJ;

  function saveQTable
    input QTableOBJ qtable;
    input String filepath;
  
    external "C" save(qtable, filepath);
  end saveQTable;

  function getAction
    input QTableOBJ qtable;
    input Integer stateDims;
    input Modelica.Blocks.Interfaces.RealInput state[stateDims];
    input Real reward;
    input Real learning_rate;
    input Real epsilon;
    input Real discount_factor;
    output Real action;
  
    external "C" action = choose_action(qtable, state, stateDims, reward, learning_rate, epsilon, discount_factor) annotation(Include = "#include \"WSM_QTable.h\"", Library = "wsm_simulations_QTable", IncludeDirectory = "modelica://wsm_simulations/../../include/wsm_simulations", LibraryDirectory = "modelica://wsm_simulations/../../../../install/lib/");
  end getAction;

  function getQValue
    input QTableOBJ qtable;
    output Real qvalue;
  
    external "C" qvalue = get_qvalue(qtable);
  end getQValue;

  function resetQtable
    input QTableOBJ qtable;
  
    external "C" reset(qtable) annotation(Include = "#include \"WSM_QTable.h\"", Library = "wsm_simulations_QTable", IncludeDirectory = "modelica://wsm_simulations/../../include/wsm_simulations", LibraryDirectory = "modelica://wsm_simulations/../../../../install/lib/");
  end resetQtable;

  Modelica.Blocks.Interfaces.IntegerOutput tableState = 0 annotation(Placement(visible = true, transformation(origin = {61.65, -62.991}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {105, -35}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput qvalue annotation(Placement(visible = true, transformation(origin = {183.679, -23.094}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {103.853, -70}, extent = {{-13.853, -13.853}, {13.853, 13.853}}, rotation = 0)));
algorithm
  when {trigger} then
    action := getAction(qtable, stateDims, state, reward, learning_rate, epsilon, discount_factor);
    qvalue := getQValue(qtable);
  end when;
equation
  if useResetPort then
    when {reset} then
      resetQtable(qtable);
    end when;
  end if;
  if saveModel then
    when sample(-0.1, savingInterval) then
      saveQTable(qtable, Modelica.Utilities.Files.loadResource(modelURI));
    end when;
  end if;
  annotation(Diagram(coordinateSystem(extent = {{-150, -90}, {150, 90}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, origin = {190, -60}, textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name", horizontalAlignment = TextAlignment.Left), Text(visible = true, origin = {0, -0.617}, textColor = {0, 128, 128}, extent = {{-103.149, -80.617}, {103.149, 80.617}}, textString = "Q", textStyle = {TextStyle.Bold}), Ellipse(visible = true, origin = {-50, -100}, lineColor = {128, 0, 0}, fillPattern = FillPattern.Sphere, extent = {{-17.33, -17.33}, {17.33, 17.33}})}));
end QTable;
