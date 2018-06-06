SPYBOT V2
===========

Le spybot de 7robot. TVN7 gare à toi!!

Comment faire fonctionner le spybot:
====================================

Pour le piloter
---------------

le minimal.launch du package spybot\_bringup permet d'avoir:
- le controle moteur
- l'odom
- la visu camera et son serveur
- L'interface web de controle

Spybot est prêts à l'emploi directement après avoir été allumer. minimal.launch est lancé automatiquement au boot ainsi qu'un serveur Apache qui vous permettra d'accéder à son interface web.

Pour le controller (au choix):
- Utiliser l'app android "ros control"
- Sur pc, executer le script remote\_control\_keyboard.sh ou le script remote\_control\_joy.sh
- Aller sur ubiquityrobot.local avec votre navigateur préféré
Soyez doux sur les commandes, la carte semble manquer de protection contre les retours de courant des moteurs...

pour faire parler le robot:
---------------------------

pour l'instant, la node de tts trouvée sur internet ne fonctionne pas. du coup il faut se connecter en ssh et taper:


```bash
echo "my english text" | festival --tts
```

Pour ajouter gmapping:
----------------------

sur le robot lancer:
- sudo ifconfig enxb827ebbebedb down; sudo ifconfig enxb827ebbebedb 169.254.157.15 up
- roslaunch sick\_tim sick\_tim551\_2050001.launch

sur un pc distant:
- lancer le launch file gmapping.launch dans le package spybot\_bringup (faudrait le deplacer dans un packege dedié)

gmapping ne fonctionne pas encore correctement! il semble qu'il y ait un probleme entre le tf du lidar et celui de la base roulante. J'avais essayé de corriger ca en modifiant les fichiers de description urdf dans le dossier sick\_tim/urdf

Archi du robot:
===============

- base roulante differentielle, avec encodeurs séparés.
- controle des moteurs via un hacheur [doc ici](http://www.bde.enseeiht.fr/clubs/robot/node/26)
- carte de puissance trouvée quelque part au fond du club, photo a venir. bref ca sort du 10v et du 5v.
- une webcam usb d'un autre temps
- un lidar tim561 [ref ici](https://www.sick.com/ag/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim561-2050101/p/p369446)

TODO
====

- Faire un asserv
- Mettre les gpio en paramètres des nodes
- Setup la navigation stack (cad ajout un path planner et un path follower)
- Améliorer l'interface web

# Contributeur
Thibaut Boissin

Thomas Chamelot
