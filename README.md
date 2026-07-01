# QoS in Unmanned Aerial Ad Hoc Network (UAANET)

Ovaj repozitorij sadrži praktični dio projekta iz predmeta **Kvaliteta usluge u telekomunikacijskim mrežama**. Projekat se bavi simulacijskom analizom rutirajućih protokola u UAV ad hoc mrežama, odnosno UAANET/FANET okruženju, korištenjem ns-3 simulatora.

Cilj projekta je porediti ponašanje različitih rutirajućih protokola u dinamičnoj mreži bespilotnih letjelica, pri čemu se posmatraju pouzdanost isporuke paketa, kašnjenje i routing overhead.

## Korišteni simulator

Simulacije su rađene u:

```text
ns-3.42
```

Repozitorij ne sadrži kompletnu ns-3 instalaciju, nego samo fajlove koji su dodani ili izmijenjeni za potrebe projekta.

## Analizirani protokoli

U simulacijama su poređeni sljedeći rutirajući protokoli:

- **AODV** – nativni ns-3 reaktivni protokol
- **OLSR** – nativni ns-3 proaktivni protokol
- **DSDV** – nativni ns-3 proaktivni distance-vector protokol
- **AODV_ETX** – modificirana verzija AODV protokola sa ETX metrikom
- **QSPU** – QoS-svjestan routing pristup zasnovan na dodatnim parametrima kvaliteta rute

## Struktura repozitorija

```text
.
├── README.md
├── scratch/
│   └── ns-3 simulacijski scenariji (.cc fajlovi)
├── src/
│   ├── aodv/
│   │   └── izmijenjeni AODV modul
│   ├── aodvetx/
│   │   └── dodani AODV_ETX modul
│   └── qspu/
│       └── dodani QSPU modul
├── scripts/
│   └── bash i Python skripte za pokretanje simulacija i obradu rezultata
├── results/
│   └── generisani CSV fajlovi i grafici
└── report/
    └── projektni izvještaj
```

## Opis simulacijskog scenarija

Simulacijski scenario predstavlja UAANET mrežu u kojoj se UAV čvorovi kreću unutar definisanog prostora, dok se GCS čvor nalazi u centralnom dijelu simulacijskog područja.

U simulacijama su analizirani sljedeći uticaji:

1. promjena broja UAV čvorova,
2. promjena brzine kretanja UAV čvorova,
3. promjena mobility modela.

Korišteni mobility modeli su:

- RandomWaypoint
- RandomWalk2D
- Gauss-Markov
- Group mobility model

Aplikacijski saobraćaj je podijeljen na više tipova:

- C2 kontrola
- C2 komanda
- telemetrija
- image saobraćaj

## Metrike

Za poređenje protokola korištene su sljedeće metrike:

- **PDR/SSDR** – omjer uspješno isporučenih paketa
- **Average delay** – prosječno end-to-end kašnjenje
- **Normalized routing overhead** – odnos routing byteova i uspješno isporučenih aplikacijskih byteova

Rezultati su dobijeni kao prosjek kroz više nezavisnih pokretanja simulacije. Korišten je fiksni RNG seed:

```text
RngSeed = 12345
```

a simulacije su pokretane za vrijednosti:

```text
RngRun = 1 do 10
```

## Kako koristiti projekat

Potrebna je čista instalacija ns-3.42. Nakon toga sadržaj ovog repozitorija treba kopirati u odgovarajuće foldere unutar ns-3.42 instalacije.

Primjer:

```bash
cp -r src/aodv /path/to/ns-3.42/src/
cp -r src/aodvetx /path/to/ns-3.42/src/
cp -r src/qspu /path/to/ns-3.42/src/

cp scratch/*.cc /path/to/ns-3.42/scratch/
cp scripts/*.sh /path/to/ns-3.42/
cp scripts/*.py /path/to/ns-3.42/
```

Zatim je potrebno buildati ns-3:

```bash
cd /path/to/ns-3.42
./ns3 build
```

## Pokretanje simulacija

Bash skripte u folderu `scripts/` korištene su za automatsko pokretanje više simulacija.

Primjer:

```bash
chmod +x *.sh

./run_uaanet_nodes_sweep.sh
./run_uaanet_speed_sweep.sh
./run_uaanet_mobility_ns3_sweep.sh
```

Ako se nazivi skripti razlikuju, potrebno je pokrenuti odgovarajuće `.sh` fajlove iz foldera `scripts/`.

## Obrada rezultata

Python skripte korištene su za obradu CSV rezultata i generisanje grafika.

Primjer:

```bash
python3 analyze_uaanet_nodes_sweep.py
python3 analyze_uaanet_speed_sweep.py
python3 analyze_uaanet_mobility_ns3_sweep.py
```

Rezultati se nalaze u folderu:

```text
results/
```

## Napomena o izmjenama u ns-3

Za potrebe projekta dodani su novi routing moduli:

```text
src/aodvetx/
src/qspu/
```

Također je izmijenjen postojeći ns-3 AODV modul:

```text
src/aodv/
```

Zbog toga je pri reprodukciji rezultata potrebno koristiti ove verzije modula, a ne samo originalne ns-3.42 fajlove.

## Zaključak

Projekat pokazuje da izbor rutirajućeg protokola u UAANET mrežama zavisi od prioriteta konkretne misije. AODV se pokazao kao stabilan osnovni izbor, OLSR često ostvaruje nizak delay, DSDV ima slabije rezultate u dinamičnim scenarijima, dok AODV_ETX i QSPU uvode dodatne informacije pri izboru rute, ali uz veći routing overhead.
