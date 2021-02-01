#include <esp8266_undocumented.h>
#include <MAX31855K.h>
bool diagPrintError(const union MAX31855K::Thermocouple& sample);
void printReport();
extern uint32_t interval_ms;

void processKey(Print& out, int hotKey) {
  switch (hotKey) {
    case 'R':
      out.printf_P(PSTR("Restart, ESP.restart(); ...\r\n"));
      ESP.restart();
      break;

    case 'p':
      out.println(F("Current MAX31855K results."));
      printReport();
      break;

    case '0':
      out.println(F("Periodic printing turned off."));
      interval_ms = 0;
      break;

    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
      interval_ms = atoi(String((char)hotKey).c_str()) * 1000;
      last = millis();
      out.printf_P(PSTR("Periodic printing set to %c second interval\r\n"), hotKey);
      break;

    case 'e':
      if (0 == max31855k.getErrorCount()) {
          out.println(F("No errors logged for MAX31855K"));
      } else {
          out.println(F("Details of last MAX31855K error."));
          diagPrintError(max31855k.getLastError());
      }
      break;

    case 'E':
      out.println(F("Clear error count."));
      max31855k.clearErrorCount();
      break;

    case '\r':
      out.println();
    case '\n':
      break;

    case '?':
      out.println();
      out.println(F("Press a key + <enter>"));
      out.println(F("  e    - Print details of last error."));
      out.println(F("  E    - Clear error count."));
      out.println(F("  p    - Print current MAX31855K results"));
      out.println(F("  0    - Turn off periodic printing"));
      out.println(F("  1    - Periodic printing every 1 second"));
      out.println(F("  2    - Periodic printing every 2 second"));
      out.println(F("  3    - Periodic printing every 3 second"));
      out.println(F("  4    - Periodic printing every 4 second"));
      out.println(F("  5    - Periodic printing every 5 second"));
      out.println(F("  6    - Periodic printing every 6 second"));
      out.println(F("  R    - Restart, ESP.restart()"));
      out.println(F("  ?    - Print Help"));
      out.println();
      out.println();
      break;
    default:
      out.printf_P(PSTR("\"%c\" - Not an option?  / ? - help"), hotKey);
      out.println();
      processKey(out, '?');
      break;
  }
}
