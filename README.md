# iRobot Roomba® 205 DustCompactor™ Combo — Webots Digital Twin

Digitale twin van de [iRobot Roomba® 205 DustCompactor™ Combo](https://www.irobot.be/nl_BE/roomba-205-dustcompactor-combo/L121240.html), ontwikkeld in het kader van de design project **Robot Design** aan de the original development environment.

De simulatie reproduceert het gedrag van het werkelijke toestel: systematische baannavigatie via ClearView™ LiDAR, automatische terugkeer naar het laadstation, modus­afwisseling tussen stofzuigen en dweilen, en tapijtvermijding.

---

## Inhoud

- [iRobot Roomba® 205 DustCompactor™ Combo — Webots Digital Twin](#irobot-roomba-205-dustcompactor-combo--webots-digital-twin)
  - [Inhoud](#inhoud)
  - [Simulatieomgeving](#simulatieomgeving)
  - [Functies](#functies)
  - [Aan de slag](#aan-de-slag)
    - [Vereisten](#vereisten)
    - [Simulatie starten](#simulatie-starten)
  - [Projectstructuur](#projectstructuur)
  - [Documentatie](#documentatie)
  - [Auteur](#auteur)

---

## Simulatieomgeving

| Component         | Versie / Waarde             |
| ----------------- | --------------------------- |
| Simulatieplatform | Webots R2025a               |
| Robotmodel        | iRobot Roomba® 205 (custom) |
| Controllertaal    | Python 3                    |

De wereld stelt een gesloten kamer voor van ±5,9 × 5,9 m met obstakels (sofa, tafel), een tapijt en een laadstation in de noordoostelijke hoek.

---

## Functies

- **Boustrophedon-navigatie** — systematische wand-tot-wand baandekking via GPS-waypoints, identiek aan het ClearView™ LiDAR-principe van het werkelijke toestel
- **Twee reinigingsmodi** — STOFZUIGEN (tapijt toegestaan) en DWEILEN (tapijtvermijding actief), met automatische fasewisseling
- **Persistente tapijt­gridkaart** — gebouwd tijdens STOFZUIGEN, herbruikt in DWEILEN voor slimme waypointgeneratie
- **Automatisch docken** — RETURNING → ALIGNING → DOCKING → CHARGING, met GPS X-correctie en meerdere fallbackcondities
- **Reactieve obstakelontwijking** — op basis van 256-ray LiDAR (180° FOV), met sector­gebaseerde snelheids- en sturingsaanpassing
- **Noodmanoeuvre (ESCAPE)** — getriggerd bij bumpercontact of afgrond­detectie, keert terug naar vorige state
- **Stuck-detectie** — automatische escape-spin als de robot langer dan 10 s minder dan 5 cm vooruitkomt
- **Persistente waypoint-afhandeling** — voltooide waypoints overleven laadcycli; de robot hervat altijd waar hij gebleven was

---

## Aan de slag

### Vereisten

- [Webots R2025a](https://cyberbotics.com/) geïnstalleerd
- Python 3.x (meegeleverd met Webots of systeeminstallatie)

### Simulatie starten

1. Kloon of download deze repository.
2. Open Webots en laad de wereld via **File → Open World**:
   ```
   worlds/project.wbt
   ```
3. Klik op **Play**. De controller start automatisch.

De robot begint in UNDOCKING-state, rijdt achteruit van het laadstation weg en start daarna de eerste reinigingsfase (STOFZUIGEN).

---

## Projectstructuur

```
project/
├── controllers/
│   └── roomba_controller/
│       └── roomba_controller.py         ← Hoofdcontroller
├── worlds/
│   └── project.wbt                      ← Webots simulatiewereld
├── docs/
│   ├── technisch_constructiedossier.md  ← Volledig technisch dossier
│   ├── Roomba_205_DustCompactor_Combo_Robot.pdf  ← Gebruikshandleiding
│   └── diagrams/
│       ├── draw.io/                     ← Bewerkbare diagrambronbestanden
│       └── images/                      ← Geëxporteerde diagramafbeeldingen
└── README.md
```

---

## Documentatie

Het technisch constructiedossier bevat een volledige beschrijving van alle sensoren, actuatoren, de state machine, de navigatiestrategie, tapijtkaartering, batterijbeheer en de softwarestructuur.

→ [`docs/technisch_constructiedossier.md`](docs/technisch_constructiedossier.md)

De officiële gebruikshandleiding van het werkelijke toestel is beschikbaar als bijlage:

→ [`docs/Roomba_205_DustCompactor_Combo_Robot.pdf`](docs/Roomba_205_DustCompactor_Combo_Robot.pdf)

---

## Auteur

**Ian Mondelaers**
