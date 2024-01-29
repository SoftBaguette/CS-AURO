1- Task definition
This assessment focuses on the retrieval of items by autonomous mobile robot(s). You will be
provided with a simulated world containing items distributed throughout the environment, which
the robot(s) must collect and return to a home zone. This simulated world will also contain
obstacles that the robot(s) need to avoid.
A robot can collect an item by driving into it, and will ‘hold’ the item until it has returned to the
home zone. A robot cannot hold more than one item at a time. Once an item has been collected
and returned home, a new one will automatically spawn to replace it.
You must design and implement a solution to this task, using mobile robot(s) that autonomously
collect the items and return them home in an efficient manner.

2- Deliverables
This assessment is made up of the following 3 parts with following weightings:

1. Implementation: 40%
2. Demonstration: 10%
3. Report: 50%

Guidance on what you are required to do for each part of the assessment is provided in the
following sub-sections.

2.1- Implementation:

You should engineer an autonomous robotic system that solves the assessment task efficiently,
while obeying the following constraints:

• You must implement your solution in simulation, using the Robot Operating System (ROS).Specifically, you must use ROS 2 Humble Hawksbill, Gazebo Classic 11, and the Python client library rclpy.

• You must use autonomously controlled (i.e. not teleoperated) TurtleBot3 Waffle Pi robot(s). Your solution may use up to 3 robot(s), and they must all start in the home zone.

• You must include a README file that describes how to run your code, and details any packages or environment variables that your implementation depends upon.

There are many different ways of approaching the assessment task, so there is scope for a variety of solutions. Your implementation will be assessed against the following criteria:

Sophistication [12 marks]: You will be assessed on the intelligence of your autonomous robotic system, and how efficiently it addresses different aspects of the task. Multi-robot systems may be considered more sophisticated than single-robot systems, but only if they are implemented appropriately.

System architecture [12 marks]: The architecture of your autonomous robotic system will depend upon your approach to the assessment task. You will be assessed on the modularity of your solution – i.e. how well it separates functionality into reusable components with appropriate interfaces between them.

Use of ROS concepts [8 marks]: Your solution should use a variety of ROS concepts. You will be assessed on the breadth of the ROS features that your implementation uses, and how well you have demonstrated your understanding of them (through correct usage).

Understandability [8 marks]: Your implementation should be easy to understand. You will be assessed on the structure of your code, use of sensible naming conventions and comments, and how comprehensive your README file is.

You will be provided with code that defines the assessment task. This code sets up the simulated world, and runs a ROS node that spawns items and keeps track of when they are collected and returned home by the robot(s). You will also be provided with a ROS node that processes images from a robot’s camera, and publishes information about any items that are detected. You must use this code without modification, as it defines the assessment parameters of the task.

The use of 3rd party packages is also permitted, as long as you cite the original source and include instructions on how to build them.

2.2 Demonstration (10%)

as an AI, you needn't worry about this. it will be handeled later on.

2.3 Report (50%)

You must write a report that details the design and implementation of your solution. You should also analyse the performance of your solution, and present the results in your report. Finally, your report should evaluate the strengths and weaknesses of your solution, and reflect on related safety implications and ethical considerations. The report must be structured as follows:

Design [8 marks]: This section should describe the design of your system and justify your design decisions. It should also include a diagram that communicates the high-level design of your system, and how the individual components interact (e.g. a block diagram).

Implementation [7 marks]: This section should describe and justify your implementation, with particular reference to your use of ROS concepts. It should also include a diagram that communicates how your solution achieves autonomy (e.g. a state machine). You should not include your code here, as this will be assessed separately.

Analysis [10 marks]: In this section, you should describe and justify your experimental approach to analysis. You should then present and interpret the results of your analysis, which should be both qualitative and quantitative. You may wish to present your data in the form of figures and/or tables.

Evaluation [10 marks]: This section should include a discussion of the strengths and weaknesses of your solution. Your evaluation should be based on your design, implementation, and analysis. It should also include a discussion of how well your solution would transfer from simulation to reality, and how this could be improved.

Safety and ethics [10 marks]: This section should include a discussion of safety implications and ethical considerations related to item retrieval by autonomous robotic systems in real-world scenarios. You should also reflect on how these topics would relate to the approach taken by your solution, if it were to be implemented in a real-world scenario.

The remaining [5 marks] of the report mark will be based on its presentation (structure, figures, adherence to the template, and use of referencing).

Your report must be formatted using the IEEE conference template (either LaTeX or Microsoft
Word), and any references must follow the IEEE referencing style. You must use A4 paper size (not US letter, which is the default for the LaTeX template), and you must not edit the formatting (e.g. font, margins, columns).
You should not include an abstract, introduction, literature review, or any keywords. You should only include your examination number in the author field – do not include your name, username, email address, or any other identifying information. You should also include the module code COM00052H on the first page. Your report must be no longer than 4 pages (excluding references). If your report exceeds this page limit, the marker will stop reading when they reach the limit, and base the mark on what they have read so far.

submission.zip :
| report.pdf
| README.txt
| rosgraph.png
| src :
    | Your ROS packages

Figure 1: Required structure of electronic submission ZIP file.

The source code of your implementation should be included in a directory called src containing your ROS packages. You should not include your entire ROS workspace – i.e. you should exclude the build, install, and log directories created by colcon. Your README file should be in plain text format. You should also include a PNG file exported from rqt_graph that shows the ROS graph of your solution. Your report must be a single PDF file.

Implementation:

Sophistication - Multi-robot system that intelligently and creatively addresses all parts of the task

System architecture - Modular architecture that separates functionality into reusable components, with considered use of interfaces

Use of ROS concepts - Demonstrates a comprehensive understanding of ROS concepts

Understandability - Exceptionally well-structured code with descriptive naming and comments where appropriate

README - Provides comprehensive instructions on how to run the code

Report:

Design - Excellend design that is well-described and thoroughly justified

Diagram - Outstanding diagram that comprehensibely communicates the design of the system

Implementation - Implementation is well-described and thoroughly justified, and demonstrates a comprehensive understanding of ROS and Outstanding diagram that comprehensively communicates the implementation of autonomy

Analysis - Rigorous experimental approach that is well-described and thoroughly justified. Insightful and thorough analysis that is both qualitative and quantitative.

Evaluation - Insightful and thorough discussion of the strengths and weaknesses of the solution. Demonstrates a comprehensive understanding of how well the solution would transfer from simulation to reality.

Safety and ethics - Insightful and thorough discussion of safety and ethics that relates these topics to the assessment
