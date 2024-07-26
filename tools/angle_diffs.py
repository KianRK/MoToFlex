import csv

def lade_winkel_aus_csv(dateipfad):
    winkel = []
    with open(dateipfad, 'r') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=";")
        for zeile in csvreader:
            # Winkelwerte für die aktuelle Zeile als float konvertieren
            try:
                winkel_zeile = [float(wert) for wert in zeile]
                winkel.append(winkel_zeile)
            except ValueError:
                # Fehlerhafte Daten überspringen oder entsprechend handhaben
                continue
    return winkel

def berechne_maximale_winkeldifferenz(winkel):
    max_diff = 0
    for i in range(1, len(winkel)):
        for j in range(len(winkel[i])):
            # Differenz für jede Spalte berechnen und mit max_diff vergleichen
            diff = abs(winkel[i][j] - winkel[i-1][j])
            max_diff = max(max_diff, diff)
    return max_diff

# Pfad zur CSV-Datei
dateipfad = '/Users/civic/dev/MoToFlex/config/UncompensatedTarget.csv'

# Winkel aus der CSV-Datei laden
winkel = lade_winkel_aus_csv(dateipfad)

# Maximale Differenz berechnen
maximale_differenz = berechne_maximale_winkeldifferenz(winkel)

print(f"Die maximale Differenz über alle Spalten beträgt: {maximale_differenz}")