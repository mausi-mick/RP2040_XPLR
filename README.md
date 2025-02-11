VL53L0X 360° Radar mit RP2040 auf Experimentierboard

Aufgebaut habe ich das „Radar" auf einem von mir entwickelten Experimentierboard für den RP2040 bzw. RP2350 im Format 100x 160.

Das Board unterstützt neben den beiden Pico-Prozessoren

    SPI - (Touch) - Displays mit ILI9341 Treiber von 2.4 bis 3.2 Zoll (240 x 320 Pixel)

                         und mit ST7736 Treiber      3.5 und 4 Zoll   (320 x 480 Pixel)

    Dual DAC MCP48x2 (8/10/12 bit)
    bis zu vier Encoder bzw. Potis
    vier Taster
    vier Steckplätze für LEDs

obige Komponenten sind auf Buchsen (Pinheader) steckbar.

Ausserdem ist das Boardmit einem kleinen Breadboard bestückt.

Die Stromversorgung erfolgt momentan über die USB-Buchse des Picos, eine zusätzliche wird auf dem Board untergebracht (umschaltbar 5V / 12V ?).

Um möglichst viele dieser Komponenten zu testen, hab ich dieses Radar auf dem Board aufgebaut.

Es besteht aus zwei VL53L0X, die mit der Rückseite zueinander auf einem kleinen Servo (SG90?) befestigt sind.

Den Drehwinkel kann man mit der Software in geringen Grenzen ändern (hier: auf 180° justieren).

Das Programm ist in der Arduino IDE (2.34) erstellt.

Für die Grafik verwende ich die hervorragende Bibliothek TFT_eSPI von H. Bodmer,

für die Distanzsensoren die Bibliothek VL53L0X.h von Pololu.

Am Anfang läuft nur in rasendem Tempo der Grafik - Test ab.

Anschliessend wird der kreisförmige Radar-Bildschirm mit den Koordinaten angezeigt. Die Distanz-Sensoren nehmen ihre Arbeit auf und das Servo dreht sich von 0° - 180° und zurück...

Die Auflösung (maximale Entfernungsanzeige ) der Graphik lässt sich mit dem Drehencoder in

Stufen von 500 mm, 1000mm, 1500 mm und 2000 mm einstellen.

Löschung des Bildschirms geschieht über den RUN (=Reset)-Pin des Pico über Taster S2.

Das Programm und die Platinen-Layouts (KiCad) werden auf Github gestellt. Es gibt auch ein

kleines KiCad-Projekt zur Beschriftung der PICO-Pins (zum Aufkleben auf dem PICO ) :

{width="17cm" height="6.93cm"}
