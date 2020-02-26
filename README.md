# rosee_gui
Gui things for [ROSEE](https://github.com/ADVRHumanoids/ROSEndEffector) project

## Requisite
QT5

## To launch
~~~~bash
roslaunch rosee_gui gui.launch
~~~~

### How it works - code structure (obviously improvable)
- **main.cpp** : It handles ROS (creating the nodehandle) and the Qapplication. It creates the Window object
- **Window.cpp** a *QWidget* derived class which refers to the gui Window. It has as member a *QGridLayout*, and it creates all the inner *QGridLayout* (one for each action)
- **ActionLayout.cpp** *QGridLayout* derived class, which contain labels buttons and other widgets. It also send message to ros topic, handling a ros publisher with a nodehandle passed to its costructor
- **ActionBoxesLayout.cpp** Derived class from above, it includes checkboxes to select finger/joint and send them also with topics. 
- **ActionTimedLayout.cpp**, **ActionTimedElement.cpp** Container and element for the timed action, which per definition is composed by other actions executed one after another with some time margins. Each element has three progress bar so in the future we can take feedback and display a progress of the action execution, included time margins
