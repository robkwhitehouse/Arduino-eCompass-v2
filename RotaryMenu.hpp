#ifndef ROTARY_MENU_HPP
#define ROTARY_MENU_HPP

/*
 * A class to model a menu that is displayed
 * on a small OLED display. The options can
 * be scrolled through and selected using
 * a rotary encoder with pushbutton
 * 
 * There are three classes in this file;
 * - RotaryMenu - base class
 * - CalibrationMenu
 * - MenuOption - struct to contain menu option data
 * 
 * RK Whitehouse - March 2022
 */

extern Adafruit_SH1106G display;
 
struct MenuOption {
  char* text;
  unsigned value;
 };

//Menu base class - not directly useable
//You must creat a child class that populates
//the menu optionList etc.

class RotaryMenu {
  public:
    void show(unsigned first);
    uint8_t currentSelection = 0;
  protected:
    char* title;
    size_t numOptions;
    MenuOption optionList[8];

};

void RotaryMenu::show(unsigned first=0) {

  //Set up OLED display
    if ( first < 0 || first > numOptions - 1) first = 0;
    uint8_t linesToShow = numOptions - first;
    if (linesToShow > 4) linesToShow = 4;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0,0);
    display.println(title);
    display.drawLine(0, 10, 127, 10, SH110X_WHITE);
    display.setCursor(0,20);
    display.print("*");   
    for (int i=0; i<linesToShow; i++) {
      display.setCursor(6,(20+i*10));
      display.println(optionList[first+i].text);
    }
    currentSelection = first;
  display.display();
 }


 //Calibration menu
class CalibrationMenu : public virtual RotaryMenu {
  public:
    CalibrationMenu() {
      title = "Calibration Menu";
      numOptions = 5;
      optionList[0] = {"Boat compass hdg.",1};
      optionList[1] = {"Reload saved offsets",2};
      optionList[2] = {"Save offsets",3};
      optionList[3] = {"Manual calibration",4};
      optionList[4] = {"Cancel and return",0};
    };
 } calibrationMenu;





#endif
