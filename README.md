

# RINS task 3 to do




## Plan:

- za task3 se map ne ujema več. Posnet nov map in ga uredit.

- morda sprememba resolutiona kamer na nekoliko boljše
- da ob srečanju facea spoznamo, če je painting ali ne, in glede na okvir to sliko izrežemo ven in shranimo. Če ni painting, pa nagovorimo, če je state 0 ali 1.
- STT za odgovor nagovarjanja.

- zaporedje prevejanja QR kode (gremo pod ring, parkiramo, v radiju dveh metrov najdemo cilinder, gremo do njega, preverimo QR kodo na njem. Če je ni, pobrišemo trenutno barvo iz self.possible_rings in gremo v phase 2. Če je pravilen, gremo v phase 3.
- da parking dokončno zanseljivo dela

- mahanje s kamero


## Implementacija:
Imejmo self.state int, ki pove, v kateri fazi smo.
Imamo set_state(int, optional_args_dict), ki nastavi state in v if elif elif... obdela to, kar se ob menjavi na state zgodi najprej.Optional args dict je recimo uporaben, ko delamo set_state(5), ker lahko podamo pozicijo prave mona lise.


0 nagovarjamo face.
: Ko dobimo namig gremo v state 1, ustvarimo self.possible_rings (prej je bil None)

1 en namig imamo, torej iščemo ringe (če je pravi, tam parkiramo) in nagovarjamo face.
: Najprej preverimo če katerega od teh že imamo in če ga res že imamo, zanj sprožimo zaporedje preverjanja QR kode (v njem je inbuilt, da če je neuspešen izbrišemo barvo iz self.possible_rings in gremo v phase 2, če je uspešen pa v phase 3).
Nadaljujemo po prejšnji navigation poti. Če najdemo enega od ringov, sprožimo zaporedje preverjanja QR kode (avtomatsko nas da v phase 2 ali 3). Če dobimo uporaben namig, posodobimo self.possible_rings in gremo state 2.

2 imamo le še en ring. Ali ker smo že sprobali en ring in cilinder ni imel QR kode, ali pa ker smo dobili drugi namig. Tu za face samo gremo zraven in jih pogledamo, in če je painting shranimo sliko, nikogar pa ne ogovarjamo.
: Najprej sprožimpo preverjanje, če smo ring te barve že videli (to nam reši primer, ko smo v state 0 že videli oba ringa - sicer do tega ne bi prišlo, ker bi tak ring že sproti obdelali).
Nadaljujemo po prejšnji navigation poti. Če vidimo ta ring, izvedemo zaporedje QR kode. Fac ne ogovarjamo več, samo shranjujemo njihove pozicije (da jih kasneje ne obiščemo ponovno) in shranjujemo slike, če so le te paintings.

3 pridobili smo pravo mona liso. Prikažemo anomalije za že dobljene paintinge. Če je en od njih pravi, gremo v state 5, sicer v state 4.

4 Nadaljujemo navigation obhod po prostoru (naj bo navigation list loop-an, da za vsak slučaj na koncu ponovimo navigation pot) in iščemo nove face in s tem pravo mona liso. Pri vsakem paintingu prikažemo anomalije. Ko jo najdemo, state 5.

5 gremo pred pravo mona liso (če smo tam je to pač idempotentno). Pomahamo s kamero.

Rings in cilindri se ves čas shranjujejo v dva dicta: {barva: marker}.
Obiskani faces in paintings se ves čas shranjujejo v en list, ki vsebuje tuples: (marker;    slika ob pozdravljanju if isPainting else None).

Callbacki za face, cilindre, ring-e vedno shranijo stvar v seznam, potem pa sprožijo action, če smo v pravem self.state.

Kar sproži callback face naj ima za ta say_hi() caviat, da to reče samo, če je self.state 0 ali 1.






## Opis naloge:


- isces cilindre + markerji
- isces ringe + markerji
- isces ljudi + markerji
- isčeš mona lise + markerji

In rviz, make markers with colours and TEXT which show what we detected.

Cilindrov si ne rabiš zapomnit na poti, ker so pomembni samo, ko si parkiral pod ring - na tega v bližini greš pogledat QR code, ki ti da mona lisa sliko.
Cilindre, ko jih srečaš po poti, si shraniš v zgodovino, da ni treba kasneje na novo iskat.
Ljudi itak takoj ogovarjaš, ko jih najdeš, tako da zanje ni log-a - samo to, ker že imamo, da ne ogovarjaš istih.
Slike si sproti shranjuješ, da lahko kasneje že preveriš, če je katera od njih prava.


- ce najdes cloveka se postavs na 0.5m pozicijo, ga prasas (TTS), odgovori (wait + STT), - analiziras odgovor
Robot shouldn't speak to a painting.
(all paintings are mona lisa paintings)
All paintings have brown border - lets us know if painting or face.

- glede na odgovor sprejmes nov goal - iskanje teh 2 ringov, ki sta v nasvetu ringa. Če je ring bil ze viden, pojdi tja, sicer naprej sledi tockam navigationa. Potem narediš parking + prebereš QR code. Če je QR code, si zmagal, sicer zdaj točno veš, pod kater ring moraš. 
2 people know about two rings (different pairs - so if you know both you have only one needed ring to park under).
Others know nothing.
On the cylinder near the correct ring the QR-code with the correct mona lisa painting is. (URL that represents it).
We will find out which of the 2 mona lisas is correct (the original, or the students recreation of mona lisa). The picture we get in advance will be the same as one of the 2 pictures we can get, so they will have difference be 0.

- ce med iskanjom pravga ringa najdes nov face, ga vprašaš za nasvet. Če je to druga oseba, ki nekaj ve, ti tudi da 2 ringa, in ta, ki je pri obeh nasvetih, je pravi ring.

- k najdes ring, parkiras (roka dol) in (roka nazaj gor) poisces cilinder v blizini
Park under ring, spins around, and should find a cylinder in the radius of 2 meters (actually, 2 units).
- k si pr cilindru skeniras qr (roka v qr pose)
- iz qr kode preberi pravo mona liso in jo displayas.

- tu potem najbolje, da za že videne slike displayamo anomalije. Če je ena od njih prava, gremo direktno tja.

- pejt od slike do slike in prever za anomalije
We get one good image, and then we create our own defects (blur, rotate, change illumination, shift changes - all possible normal images of mona lisa).
Then create test images (they will also make them now to make our life easier). But they will put them in the polygon, so we should drive it around and get 10 pictures of each given painting).
If there is a defect, it should be displayed.
Surface anomaly detection, da vidiš, če je slika enaka tej ta pravi.
A simple PCA should work.

- ce je slika original, pomahas z roko
Weaving with the manipulator is a point - ig moving the camera you have on top.
- ce ni, prikazes anomalijo


- autonomous poisce + poveze tocke (2 points out of 30)














































# Staro za RINS-task-2:

### Spreminjanje parametrov stene:
V config/nav2.yaml greš pod:
local_costmap:   local_costmap:    ros_parameters:   inflation_layer:
in spremeniš 
cost_scaling_factor (originalno 4.0)
inflation_radius  (originalno 0.45)

## To do:

- nardit v robotcommanderju subscriberja na te detectione in da se odziva na to in potuje
- nardit za detectione da se ne ponavlja detection, če je na istem mestu.
- text to speech

- izboljšat ta opening in closing. Ali pa mogoče celo tresholding. Nekako je treba to držalno roko odrstranit brez da preveč deformiraš ring da bi bil zaznan.
(Basically, nimamo težav s false positives, ampak ne dobimo teh ringov ffs)
Kot kaže je ves problem ring detectiona v tej roki zraven, ker ko je lepo closed away, deluje super.

- preverit če moje zaznavanje globine sredine sploh dela.
Do sedaj ni problema s false detectioni flying kvadrov.
Sprobati ta depth detection, da kakšne te blocke v zraku ne misidentifyamo.
Moj depth checking ni prav preverjen, ker ga nisem mogel zganjat. Poglej, da nisem kje narobe obrnil koordinat slike.
funkcijo get_depth...preveri

Z zmanjšanje false positives ring detectiona (ne rabimo)
- axis se med manjšim in večjim ne smejo za veliko razlikovat.



- text to speech
- detektiranje cilindrov (z barvo) robustifikacija, subscription, prependanje teh goalov
- Detektiranje krogov (z barvo, pa ne flat krogov torej preverit depth na krogu in v sredini in videt če sta različna (pazi na infinities in take stvari ko v ozadju ni stene ampak air)), subscription, prependanje teh goalov - če je zelen, potem še prependanje goala servoing mehanizma
- servoing mehanizem, kjer gledamo za krog na tleh in probamo bit celi noter.
Ideji asistenta na vajah:
- z zgornjo kamero (na robotski roki) pogledaš blob in s cv2 dobiš njegov center (blob je ta lunca ki jo omejujeta t in vidni del circlea - ta notranjost). In se obrneš proti temu centru in potuješ v tisto smer. In ko je blob dovolj simetričen in ko je center bloba dost v redu, ali pa ko je dost thin, takrat si zmagal.
- pogledaš vidni ring in fittaš hough transform, da dobiš center kroga. Ker pravokotno na tla gledaš pomoje lahko celo circle fittaš, ne elipse. In lepo dobiš center na image coordinates. Potem pa imamo narejeno že nmeko transformacijo iz image v world coordinates. In se premikaš proti centru dokler napaka ni dovolj majhna. Ampak ta transformacija je nekoliko nerobustna, zato asisten predlaga prvo opcijo.

### Postransko:
- narediti da say_hi() deluje tudi če zaženemo iz drugje kot ~/ros_ws
- levo zgoraj (ob koncu predpisane poti) se robot zatakne. Spremenit bi bilo treba severity levels ste al kaj ze, pomoje nekaj takega. Da si bo pac upal it skoz.
- če je v desnem koridorju in mora it na base position se tudi rado zatakne. Sprememba severity levels v launchu bi znala pomagat. Ali pa nek manever reševanja, če je predolgo statičen.
- spreminjanje hitrosti (ima prehitrost vpliv na zaznavo? Če nima, bi šli morda lahko hitreje? Zdi se, da bi se ta max_velocity dal v config/nav2.yaml naštimat)
- se da narediti, da pri navigiranju pose objectu ne damo fi-ja, in pač pusti robota v katerikoli smeri je pac obrnjen ko pride do ciljne točke? In ga mi potem recimo obrnemo za 360, da pogledamo za obraze, in gre potem naprej - ker zdaj se ze toliko obraca, da ga je prav wasteful it 360 obracat.
- either way, lahko bi rocno dodali obracanja za vsako mesto postanka posebej in bi se actually obrnil proti stenam, ne vseh 360.