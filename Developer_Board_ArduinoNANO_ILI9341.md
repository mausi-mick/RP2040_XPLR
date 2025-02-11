.

`    `**Test Board für Arduino „NANOs“ mit ILI9341, ILI9488-bzw. ST7735 SPI-Display**

Bei vielen meiner Projekte  verwende ich den Arduino NANO oder Pin-kompatible Prozessoren mit dem TFT-Farb-Display ILI9341 bzw. ILI9488 bzw. ST7735 aus folgenden Gründen:



- Existenz vieler Bibliotheken (libs) mit zahlreichen Beispielen für Prozessor und Display
- kompaktes Board mit relativ grosser Anzahl herausgeführter Pins für andere Funktionen,
- leicht beschaffbar / preisgünstig
- Existenz von „NANO“  Boards mit anderem Processor mit NANO-“Pin-mapping“
- mit HW-SPI ist das Display -auch am NANO – relativ schnell
- die SPI-Displays ILI9341, ILI9488 und ST7735 haben gleiche/ähnliche Pinbelegung
- unterschiedliche Displaygrössen (1.8“,2,4“,2.8“,3.2“.3.5“,4“) (mit/ohne Touch) erhältlich
- unterschiedliche Auflösungen (128x160), (240x320), (320x 400)

mittlerweile sind zum Arduino-NANO pinkompatible Boards erhältlich, z.B. :



- **LGT8F328P----»M**ini EVB Nano styl**e“,** Prozessor: **modifzierter ATMega328**
- **Arduino EVERY:**                                     Prozessor**: ATMega4809**
- **Arduino NANO ESP32                           ,**Prozessor: **ESP32**
- **Arduino NANO 33 BLE                         ,**Prozessor: **nRF52840**
- **Arduino NANO 33 IOT                         ,** Prozessor: **M0+SAMD21**
- **Arduino NANO RP2040 Connect          ,**Prozessor: **RP2040**

Die neueren Boards sind dann eher auf 3,3 V Logik ausgerichtet bzw. z.T. umlötbar.

Als Entwickler möchte ich eigentlich möglichst schnell zur Anwendung kommen und mich nicht

mit den Logik-Leveln rumschlagen , sondern den Prozessor und das Display auf ein

Board setzen und mit der Entwicklung loslegen., möglichst ohne viele Leitugen zu verlegen.

Daher habe ich dieses Board entwickelt, auf dem Steckfassungen für das Display und das „NANO“-Board angeordnet sind und zusätzlich noch Steck Buchsen für:

- einen DUAL-DAC MCP48x2
- bis zu vier Drehencoder ( bzw. Potis),
- vier Drucktaster
- vier LED Anzeigen
- ein Mini-Breadboard
- negative Sapnnungsquelle(-5V / ca 100mA) für Oper.verstärker

Das SPI-Display ( ILI9341, ILI9488, ST7735) wird direkt auf das Board gesteckt , auf dem sich die Verdrahtung für die Spannungsversorgung und die SPI-Anschlüsse (CS (Chipselect) an D10, DC (Data/Control) an D9)  des Displays befindet. Die Touch Pins und die SD-Karten Pins des Displays sind auf Buchsenleisten herausgeführt, ebenso wie alle Prozessor-Pins.

Es sind Displays in den Grössen 1.8“(ST7735), 2.4, 2.8 , 3.2“ (ILI9341) bzw. 3.5, 4.0“ (ILI9488) verwendbar.


Alles ist auf einer einlagigen Platine im Europa-Format (160x100) untergebracht.

Die Stromversorgung des Boards und die Programmierung erfolgt vom PC / Notebook über ein USB-Kabel, der Strom ist mit  500 mA abgesichert.

Ist die Programmierung abgeschlossen, kann das Board auch über eine USB Powerbank betrieben werden.

Nur für grössere Verbraucher (Servos, Motoren, Relais,…) wird eine zusätzliche entsprechend

dimensionierte Spannungsversorgung benötigt.

Von diesem „Main“ Board sind die „NANO“ Boards zur Programmierung über kurze USB Kabel verbunden, je nach Board mit Mini-USB (NANO), Micro-USB (Arduino EVERY, LTG8F328) oder

USB-C (LGT8F328 (blaue Platine),.

Zu dem Board gibt es KiCad-files (KiCad 7.0) und ein Blockdiagramm (DrawIO), auch als pdf.

Ausserdem gibt es noch zwei Adapter-Platinen für

den  

`            `**Adafruit-Metro Mini 328**



und den

`            `**Raspberry PICO RP2040 (W)**

`  `Ausserdem werden Testprogramme zu den einzelnen Boards und Displays beigefügt.


`   `**Hardware-Komponenten**

*hier sind jetzt nur die wesentlichen Komponenten / Module aufgeführt, die Details kann man den KiCad Stücklisten entnehmen.*

- *ILI9341 SPI-Display (2.4“, 2.8“ oder 3.2“) (Touch bzw ohne Touch) (240x320 Pixel)*
- *bzw ILI9488 Display (3,5“ bzw. 4.0“)  (Touch bzw ohne Touch) (320x400 Pixel)*
- *bzw. ST7735 Display( 1.8“)  ohne Touch (128x160 Pixel)*
- *Arduino NANO  oder Varianten gleicher Pin-Belegung*
- *USB C Buchse zur Verbindung PC/Notebbok*
- *USB-A-Buchsezur Verbindung zum Prozesorboard*
- *je nach Prozessorboard kurzes USB-Kabel mit mini-, micro USB-Stecker / USB-C-Stecker*
- *diverse Buchsenleisten Raster 2.54 mm*
- *LDO variabel ( MC33269) für 3.7V aus 5V*
- *Schiebeschalter/Kippscahlter 1x UM)*
- *diverse Widerstände, Kondenatoren ,  Dioden und Sicherungen*
- *optional:*
- `   `*DualDAC MCPM4802, MCP4812, MCP4822 ( 8, 10,12 bit SPI*
- `   `*1x DIL8-Fassung*
- `   `*4xButtonSwitch*
- `   `*4xLeds*
- `   `*Dreh-Encoder (0-4 Stück)  ??*
- `   `*Mini-Bread-Board*
- `   `*-negatve Spannungsverorgung mit ICL7662 Modul*
- `  `*Adapterboards für Adafruit Metro-Mini 328 / Raspberry PI RP2040*

























\*



