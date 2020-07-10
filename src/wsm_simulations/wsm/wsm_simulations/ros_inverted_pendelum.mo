within wsm_simulations;

model ros_inverted_pendelum
  IntroductoryExamples.Systems.Components.ElectricalMotor motor annotation(Placement(visible = true, transformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  IntroductoryExamples.Systems.Components.Pendulum pendulum annotation(Placement(visible = true, transformation(origin = {90, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  IntroductoryExamples.Systems.Components.GearBox gear annotation(Placement(visible = true, transformation(origin = {50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.ROS_Node rOS_Node1(node_name = "inverted_pendelum_simulation", SampleRate = 50) annotation(Placement(visible = true, transformation(origin = {-102.242, 12.797}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Subscriber subscriber1(topic = "pendelum_motor") annotation(Placement(visible = true, transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Publisher publisher1(topic = "pendelum_state", datatype = "wsm_simulations/PendelumState", pythonSitePackages = "C:\\Users\axelw\\github\\ROSPlug_tests\\install\\lib\\site-packages") annotation(Placement(visible = true, transformation(origin = {7.5, -5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Readers.RealReader realReader1(path = "data") annotation(Placement(visible = true, transformation(origin = {-25, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Writers.RealWriter realWriter1(path = "angle") annotation(Placement(visible = true, transformation(origin = {85, 7.129}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Writers.RealWriter realWriter2(path = "position") annotation(Placement(visible = true, transformation(origin = {85, -13.389}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.SampleTrigger sampleTrigger1(period = 0.1) annotation(Placement(visible = true, transformation(origin = {5, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.SampleTrigger sampleTriggerOffset(period = 0.1, startTime = 0.05) annotation(Placement(visible = true, transformation(origin = {5, -37.33}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(motor.flange, gear.flange_a) annotation(Line(visible = true, points = {{-10, 0}, {10, 0}}, color = {64, 64, 64}, origin = {30, 40}));
  connect(gear.flange_b, pendulum.flange_a) annotation(Line(visible = true, origin = {71.034, 40}, points = {{-11.034, 0}, {8.966, 0}}, color = {0, 127, 0}));
  connect(subscriber1.subscriberPublisherConnector1, rOS_Node1.rosNodeConnector1) annotation(Line(visible = true, origin = {-77.024, -50}, points = {{6.524, 90}, {-2.976, 90}, {-2.976, 62.797}, {-16.017, 62.797}}));
  connect(publisher1.subscriberPublisherConnector1, rOS_Node1.rosNodeConnector1) annotation(Line(visible = true, origin = {-78.769, -40}, points = {{75.769, 35}, {-1.231, 35}, {-1.231, 52.797}, {-14.272, 52.797}}));
  connect(subscriber1.message1, realReader1.messageInConnector1) annotation(Line(visible = true, origin = {-42.39, 40}, points = {{-8.075, -0}, {8.075, 0}}, color = {64, 64, 64}));
  connect(realReader1.y, motor.u) annotation(Line(visible = true, origin = {6, 51.321}, points = {{-21, -11.321}, {-18.409, -11.321}, {-18.409, -11.321}, {-7, -11.321}}, color = {1, 37, 163}));
  connect(pendulum.angle, realWriter1.y) annotation(Line(visible = true, origin = {94, 17.5}, points = {{7, 17.5}, {11, 17.5}, {11, -10.371}, {1, -10.371}}, color = {1, 37, 163}));
  connect(pendulum.position, realWriter2.y) annotation(Line(visible = true, origin = {101.5, 10}, points = {{-0.5, 35}, {18.5, 35}, {18.5, -23.389}, {-6.5, -23.389}}, color = {1, 37, 163}));
  connect(realWriter2.messageOutConnector1, publisher1.message1) annotation(Line(visible = true, origin = {58.207, -8.461}, points = {{17.203, -4.928}, {11.793, -4.929}, {11.793, 4.929}, {-40.788, 4.929}}, color = {64, 64, 64}));
  connect(realWriter1.messageOutConnector1, publisher1.message1) annotation(Line(visible = true, origin = {58.207, 1.798}, points = {{17.203, 5.331}, {11.793, 5.33}, {11.793, -5.33}, {-40.788, -5.33}}, color = {64, 64, 64}));
  connect(sampleTrigger1.y, realWriter2.trigger) annotation(Line(visible = true, origin = {70.25, -31.194}, points = {{-54.25, -38.805}, {44.75, -38.805}, {44.75, 13.805}, {24.75, 13.805}}, color = {255, 0, 255}));
  connect(sampleTrigger1.y, realWriter1.trigger) annotation(Line(visible = true, origin = {70.25, -20.935}, points = {{-54.25, -49.064}, {44.75, -49.064}, {44.75, 24.064}, {24.75, 24.064}}, color = {255, 0, 255}));
  connect(sampleTriggerOffset.y, publisher1.trigger) annotation(Line(visible = true, origin = {23.336, -19.5}, points = {{-7.336, -17.83}, {6.664, -17.83}, {6.664, 10.5}, {-5.991, 10.5}}, color = {255, 0, 255}));
  annotation(experiment(Interval = 0.004, __Wolfram_SynchronizeWithRealTime = true), Diagram(coordinateSystem(extent = {{-150, -90}, {150, 90}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {0, 114, 195}, fillColor = {255, 255, 255}, extent = {{-100, -100}, {100, 100}}, radius = 25), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name")}));
end ros_inverted_pendelum;
