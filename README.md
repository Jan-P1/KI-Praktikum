# KI-Team-16

## Idee: 001

### Variablen count: 2
 1. Variable ist foat von 0.00 bis 1.00, wenn auf 0.00 so 
wird IMMER die rute mit dem niedrigsten Kanten gewicht genommen, auf 1.00
immer das höchste gewicht. initialisierung setzt den wert FEST,
kann einen 0.05 mit 0.15 zu 0.10 vererben. 
Beispiel: 0.20 bei --> 1,2,3,4,5,6,7,8,9,10 = 2 oder der bei 20% zu dem niedrigsten wert
2. Bei uneindeutigkeit (2,2,2 etc.) ZUFÄLLIGE auswahl aus kern auswahl
### Methoden count: 0



## Idee: 002
### Variablen count: 2
1. speicher grade genommenes kantengewicht
2. Prozentzahl (kann auch 500% betragen (5.00))
### Methoden count: 1
fergleicht kanntengewicht der zuletzt beschrittenen kante mit 
offenen neuen möglichkeiten, wenn zuletzt beschrittenens kanntengewicht um 2.(%) kleiner als alle nun vorhandenen neuen Kanten ist, wird ein schritt zurück getahen und die nächst kleine kannte beschritten... etc. rinc and repeat
Vererbungsschritt = anpassung der Prozentzahl -> führt zu unterschiedlich häufigen rückschritten



 

## Idee: 003
### Variablen count: 2
1. die X te
2. von 'links' / von 'rechts' (boolean= true -> links)

### Methoden count: 1
nimm immer von den möglich nehmbaren ruten jene, die nach sortierung die X te von 'links' (klein nach groß)/ von 'rechts' (groß nach klein) 
