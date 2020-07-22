within wsm_simulations;

model ros_inverted_pendelum_relay
  IntroductoryExamples.Systems.Components.ElectricalMotor motor annotation(Placement(visible = true, transformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  IntroductoryExamples.Systems.Components.Pendulum pendulum annotation(Placement(visible = true, transformation(origin = {90, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  IntroductoryExamples.Systems.Components.GearBox gear annotation(Placement(visible = true, transformation(origin = {50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.ROS_Node rOS_Node1(node_name = "inverted_pendelum_simulation", SampleRate = 100) annotation(Placement(visible = true, transformation(origin = {-102.242, 12.797}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Subscriber subscriber1(topic = "pendelum_motor") annotation(Placement(visible = true, transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Publisher publisher1(topic = "pendelum_state", datatype = "wsm_simulations/PendelumState", msgSitePackageURI = "modelica://wsm_simulations/../../../../devel/lib/site-packages", internalMessagingPolicy = ROSPlug.Publisher.InternalMessagingPolicy.Only) annotation(Placement(visible = true, transformation(origin = {33.655, -15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Readers.RealReader motor_response(path = "motor_response") annotation(Placement(visible = true, transformation(origin = {-25, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Writers.RealWriter writerVelocity(path = "velocity") annotation(Placement(visible = true, transformation(origin = {85, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Writers.RealWriter writerPosition(path = "position") annotation(Placement(visible = true, transformation(origin = {85, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.SampleTrigger sampleTrigger1(period = period) annotation(Placement(visible = true, transformation(origin = {30, -85}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Readers.RealReader average_reward(path = "average_reward") annotation(Placement(visible = true, transformation(origin = {-25, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Readers.IntegerReader explored_states(path = "explored_states") annotation(Placement(visible = true, transformation(origin = {-25, 15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Writers.RealWriter writerDeltaTime(path = "delta_time") annotation(Placement(visible = true, transformation(origin = {85, -95}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression realExpression1(y = period) annotation(Placement(visible = true, transformation(origin = {85, -115}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Modelica.SIunits.Time period(min = Modelica.Constants.small, start = 0.01) = 0.2 "Sample period (sampleTriggerOffset.period)";
  ROSPlug.Readers.RealReader qvalue(path = "qvalue") annotation(Placement(visible = true, transformation(origin = {-25, -33.034}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.SampleTrigger sampleTrigger2(period = period, startTime = 0.1) annotation(Placement(visible = true, transformation(origin = {31.686, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Writers.RealWriter writerAngularVelocity(path = "angular_velocity") annotation(Placement(visible = true, transformation(origin = {85, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROSPlug.Writers.RealWriter writerAngle(path = "angle") annotation(Placement(visible = true, transformation(origin = {85, -45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Derivative derivative1 annotation(Placement(visible = true, transformation(origin = {150, 45}, extent = {{-6.612, -6.612}, {6.612, 6.612}}, rotation = 0)));
  Modelica.Blocks.Continuous.Derivative derivative2 annotation(Placement(visible = true, transformation(origin = {118.388, 35}, extent = {{-6.612, -6.612}, {6.612, 6.612}}, rotation = 0)));
  Modelica.Blocks.Math.WrapAngle wrapAngle1 annotation(Placement(visible = true, transformation(origin = {106.334, 16.334}, extent = {{-6.334, -6.334}, {6.334, 6.334}}, rotation = -90)));
equation
  connect(motor.flange, gear.flange_a) annotation(Line(visible = true, points = {{-10, 0}, {10, 0}}, color = {64, 64, 64}, origin = {30, 40}));
  connect(gear.flange_b, pendulum.flange_a) annotation(Line(visible = true, origin = {71.034, 40}, points = {{-11.034, 0}, {8.966, 0}}, color = {0, 127, 0}));
  connect(subscriber1.subscriberPublisherConnector1, rOS_Node1.rosNodeConnector1) annotation(Line(visible = true, origin = {-77.024, -50}, points = {{6.524, 90}, {-2.976, 90}, {-2.976, 62.797}, {-16.017, 62.797}}));
  connect(publisher1.subscriberPublisherConnector1, rOS_Node1.rosNodeConnector1) annotation(Line(visible = true, origin = {-78.769, -40}, points = {{101.924, 25}, {78.769, 25}, {78.769, -8.315}, {-1.231, -8.315}, {-1.231, 52.797}, {-14.272, 52.797}}));
  connect(subscriber1.message1, motor_response.messageInConnector1) annotation(Line(visible = true, origin = {-42.39, 40}, points = {{-8.075, -0}, {8.075, 0}}, color = {64, 64, 64}));
  connect(motor_response.y, motor.u) annotation(Line(visible = true, origin = {6, 51.321}, points = {{-21, -11.321}, {-18.409, -11.321}, {-18.409, -11.321}, {-7, -11.321}}, color = {1, 37, 163}));
  connect(pendulum.position, writerPosition.y) annotation(Line(visible = true, origin = {101.5, 0}, points = {{-0.5, 45}, {33.5, 45}, {33.5, -70}, {-6.5, -70}}, color = {1, 37, 163}));
  connect(writerPosition.messageOutConnector1, publisher1.message1) annotation(Line(visible = true, origin = {64.746, -41.766}, points = {{10.664, -28.234}, {5.254, -28.234}, {5.254, 28.234}, {-21.172, 28.234}}, color = {64, 64, 64}));
  connect(writerVelocity.messageOutConnector1, publisher1.message1) annotation(Line(visible = true, origin = {64.746, -4.266}, points = {{10.664, 9.266}, {5.254, 9.266}, {5.254, -9.266}, {-21.172, -9.266}}, color = {64, 64, 64}));
  connect(sampleTrigger1.y, writerPosition.trigger) annotation(Line(visible = true, origin = {70.25, -41.194}, points = {{-29.25, -43.806}, {44.75, -43.806}, {44.75, -32.806}, {24.75, -32.806}}, color = {255, 0, 255}));
  connect(sampleTrigger1.y, writerVelocity.trigger) annotation(Line(visible = true, origin = {70.25, -30.935}, points = {{-29.25, -54.065}, {44.75, -54.065}, {44.75, 31.935}, {24.75, 31.935}}, color = {255, 0, 255}));
  connect(subscriber1.message1, average_reward.messageInConnector1) annotation(Line(visible = true, origin = {-42.432, 55}, points = {{-7.873, -15}, {-2.568, -15}, {-2.568, -65}, {8.117, -65}}, color = {64, 64, 64}));
  connect(subscriber1.message1, explored_states.messageInConnector1) annotation(Line(visible = true, origin = {-42.3, 27.5}, points = {{-8.005, 12.5}, {-2.7, 12.5}, {-2.7, -12.5}, {7.985, -12.5}}, color = {64, 64, 64}));
  connect(writerDeltaTime.messageOutConnector1, publisher1.message1) annotation(Line(visible = true, origin = {64.746, -54.266}, points = {{10.664, -40.734}, {5.254, -40.734}, {5.254, 40.734}, {-21.172, 40.734}}, color = {64, 64, 64}));
  connect(sampleTrigger1.y, writerDeltaTime.trigger) annotation(Line(visible = true, origin = {85.25, -64.5}, points = {{-44.25, -20.5}, {29.75, -20.5}, {29.75, -34.5}, {9.75, -34.5}}, color = {255, 0, 255}));
  connect(realExpression1.y, writerDeltaTime.y) annotation(Line(visible = true, origin = {107.75, -55}, points = {{-11.75, -60}, {12.25, -60}, {12.25, -40}, {-12.75, -40}}, color = {1, 37, 163}));
  connect(publisher1.trigger, sampleTrigger2.y) annotation(Line(visible = true, origin = {50.186, -38.162}, points = {{-6.686, 19.162}, {9.814, 19.162}, {9.814, -11.838}, {-7.5, -11.838}}, color = {190, 52, 178}));
  connect(writerAngle.messageOutConnector1, publisher1.message1) annotation(Line(visible = true, origin = {64.746, -29.266}, points = {{10.664, -15.734}, {5.254, -15.734}, {5.254, 15.734}, {-21.172, 15.734}}, color = {64, 64, 64}));
  connect(writerAngularVelocity.messageOutConnector1, publisher1.message1) annotation(Line(visible = true, origin = {64.746, -16.766}, points = {{10.664, -3.234}, {5.254, -3.234}, {5.254, 3.234}, {-21.172, 3.234}}, color = {64, 64, 64}));
  connect(sampleTrigger1.y, writerAngle.trigger) annotation(Line(visible = true, origin = {85.883, -67.5}, points = {{-44.883, -17.5}, {29.117, -17.5}, {29.117, 18.5}, {9.117, 18.5}}, color = {255, 0, 255}));
  connect(sampleTrigger1.y, writerAngularVelocity.trigger) annotation(Line(visible = true, origin = {85.25, -56.143}, points = {{-44.25, -28.857}, {29.75, -28.857}, {29.75, 32.143}, {9.75, 32.143}}, color = {255, 0, 255}));
  connect(pendulum.position, derivative1.u) annotation(Line(visible = true, origin = {121.533, 45}, points = {{-20.533, 0}, {20.533, 0}}, color = {1, 37, 163}));
  connect(derivative1.y, writerVelocity.y) annotation(Line(visible = true, origin = {144.205, 25}, points = {{13.068, 20}, {18.068, 20}, {18.068, -20}, {-49.205, -20}}, color = {1, 37, 163}));
  connect(pendulum.angle, derivative2.u) annotation(Line(visible = true, origin = {105.726, 35}, points = {{-4.726, 0}, {4.726, 0}}, color = {1, 37, 163}));
  connect(derivative2.y, writerAngularVelocity.y) annotation(Line(visible = true, origin = {120.496, 7.5}, points = {{5.165, 27.5}, {10.165, 27.5}, {10.165, -27.5}, {-25.496, -27.5}}, color = {1, 37, 163}));
  connect(subscriber1.message1, qvalue.messageInConnector1) annotation(Line(visible = true, origin = {-43.235, 3.483}, points = {{-7.07, 36.517}, {-1.765, 36.517}, {-1.765, -36.517}, {8.921, -36.517}}, color = {64, 64, 64}));
  connect(wrapAngle1.u, pendulum.angle) annotation(Line(visible = true, origin = {104.556, 31.312}, points = {{1.778, -7.376}, {1.778, 3.688}, {-3.556, 3.688}}, color = {1, 37, 163}));
  connect(wrapAngle1.y, writerAngle.y) annotation(Line(visible = true, origin = {102.556, -26.878}, points = {{3.778, 36.244}, {3.778, -18.122}, {-7.556, -18.122}}, color = {1, 37, 163}));
  annotation(experiment(StopTime = 100000.0, Interval = 0.004, __Wolfram_SynchronizeWithRealTime = true), __Wolfram, Diagram(coordinateSystem(extent = {{-122.183, -140}, {176.835, 90}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name"), Rectangle(visible = true, lineColor = {191, 191, 191}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, lineThickness = 4, extent = {{-90, -90}, {90, 90}}, radius = 25), Rectangle(visible = true, origin = {-4.648, 24.511}, rotation = 5, lineColor = {0, 67, 109}, fillColor = {169, 0, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, lineThickness = 0, extent = {{-4.67, -48.292}, {4.67, 48.292}}), Rectangle(visible = true, origin = {0, -50}, fillColor = {169, 0, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, lineThickness = 0, extent = {{-40, -20}, {40, 20}}), Ellipse(visible = true, origin = {-0.397, -25}, fillColor = {169, 0, 0}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-5, -5}, {5, 5}}), Rectangle(visible = true, origin = {26.323, 31.62}, lineColor = {128, 128, 128}, fillColor = {0, 0, 128}, fillPattern = FillPattern.Solid, extent = {{-66.667, -13.743}, {13.333, 12.827}}), Polygon(visible = true, origin = {18.725, 32}, lineColor = {128, 128, 128}, fillColor = {0, 0, 128}, fillPattern = FillPattern.Solid, points = {{20, 28}, {81.275, 0}, {20, -28}, {20, -28}}), Polygon(visible = true, origin = {-18.681, 29.877}, rotation = -180, lineColor = {128, 128, 128}, fillColor = {0, 0, 128}, fillPattern = FillPattern.Solid, points = {{20, 28}, {81.275, 0}, {20, -28}, {20, -28}})}));
end ros_inverted_pendelum_relay;
