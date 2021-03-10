# -*- coding: UTF-8 -*-

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
'                                                                         '
' Copyright 2018-2021 Gauthier Brière (gauthier.briere "at" gmail.com)    '
'                                                                         '
' This file is part of cn5X++                                             '
'                                                                         '
' cn5X++ is free software: you can redistribute it and/or modify it       '
' under the terms of the GNU General Public License as published by       '
' the Free Software Foundation, either version 3 of the License, or       '
' (at your option) any later version.                                     '
'                                                                         '
' cn5X++ is distributed in the hope that it will be useful, but           '
' WITHOUT ANY WARRANTY; without even the implied warranty of              '
' MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           '
' GNU General Public License for more details.                            '
'                                                                         '
' You should have received a copy of the GNU General Public License       '
' along with this program.  If not, see <http://www.gnu.org/licenses/>.   '
'                                                                         '
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import sys, time
import serial
from enum import Enum
from math import *
from PyQt5.QtCore import QCoreApplication, QObject, QThread, QTimer, QEventLoop, pyqtSignal, pyqtSlot, QIODevice
from .cn5X_config import *
from .grblComStack import grblStack


class grblComSerial(QObject):
  '''
  QObject worker assurant la gestion de la communication serie bidirectionnelle entre cn5X++ et grbl.
  Doit etre execute dans son propre thread pour ne pas bloquer l'interface graphique.
  '''

  sig_connect    = pyqtSignal(bool)     # Message emis a la connexion (valeur = True) et a la deconnexion ou en cas d'erreur de connexion (valeur = False)
  sig_log        = pyqtSignal(int, str) # Message de fonctionnement du composant grblComSerial, renvoie : logSeverity, message string
  sig_init       = pyqtSignal(str)      # Emis a la reception de la chaine d'initialisation de Grbl, renvoie la chaine complete
  sig_ok         = pyqtSignal()         # Emis a la reception de la chaine "ok"
  sig_error      = pyqtSignal(int)      # Emis a la reception d'une erreur Grbl, renvoie le N° d'erreur
  sig_alarm      = pyqtSignal(int)      # Emis a la reception d'une alarme Grbl, renvoie le N° d'alarme
  sig_status     = pyqtSignal(str)      # Emis a la reception d'un message de status ("<...|.>"), renvoie la ligne complete
  sig_config     = pyqtSignal(str)      # Emis a la reception d'une valeur de config ($XXX)
  sig_data       = pyqtSignal(str)      # Emis a la reception des autres donnees de Grbl, renvoie la ligne complete
  sig_probe      = pyqtSignal(str)      # Emis a la reception d'un résultat de probe
  sig_emit       = pyqtSignal(str)      # Emis a l'envoi des donnees sur le port serie
  sig_recu       = pyqtSignal(str)      # Emis a la reception des donnees sur le port serie
  sig_debug      = pyqtSignal(str)      # Emis a chaque envoi ou reception
  sig_activity   = pyqtSignal(bool)     # Emis lors de l'émission/réception de données sur le port série
  sig_serialLock = pyqtSignal(bool)     # Emis a chaque changement de self.__okToSendGCode

  def __init__(self, decodeur, comPort: str, baudRate: int, pooling: bool):
    super().__init__()
    self.__decode = decodeur

    self.__abort            = False
    self.__portName         = comPort
    self.__baudRate         = baudRate

    self.__realTimeStack    = grblStack()
    self.__mainStack        = grblStack()

    self.__initOK           = False
    self.__grblStatus       = ""

    self.__queryCounter     = 0
    self.__querySequence    = [
      REAL_TIME_REPORT_QUERY,
      REAL_TIME_REPORT_QUERY,
      REAL_TIME_REPORT_QUERY,
      REAL_TIME_REPORT_QUERY,
      CMD_GRBL_GET_GCODE_STATE + '\n'
    ]
    self.__lastQueryTime    = time.time()
    self.__pooling          = pooling
    self.__okToSendGCode = True
    self.sig_serialLock.emit(self.__okToSendGCode)


  @pyqtSlot()
  def startPooling(self):
    self.__pooling = True


  @pyqtSlot()
  def stopPooling(self):
    self.__pooling = False


  @pyqtSlot()
  def abort(self):
    ''' Traitement du signal demandant l'arret de la communication '''
    self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial.py: abort received."))
    self.__abort = True


  @pyqtSlot()
  def clearCom(self):
    ''' Vide les files d'attente '''
    self.__realTimeStack.clear()
    self.__mainStack.clear()


  @pyqtSlot(str)
  @pyqtSlot(str, object)
  def realTimePush(self, buff: str, flag = COM_FLAG_NO_FLAG):
    ''' Ajout d'une commande GCode dans la pile en mode FiFo '''
    self.__realTimeStack.addFiFo(buff, flag)


  @pyqtSlot(str)
  @pyqtSlot(str, object)
  def gcodePush(self, buff: str, flag = COM_FLAG_NO_FLAG):
    ''' Ajout d'une commande GCode dans la pile en mode FiFo (fonctionnement normal de la pile d'un programe GCode) '''
    self.__mainStack.addFiFo(buff, flag)


  @pyqtSlot(str)
  def resetSerial(self):
    ''' Reinitialisation de la communication série '''
    self.__realTimeStack.clear()
    self.__mainStack.clear()
    self.__sendData(REAL_TIME_SOFT_RESET)
    self.__okToSendGCode = True
    self.sig_serialLock.emit(self.__okToSendGCode)


  @pyqtSlot(str)
  @pyqtSlot(str, object)
  def gcodeInsert(self, buff: str, flag = COM_FLAG_NO_FLAG):
    ''' Insertion d'une commande GCode dans la pile en mode LiFo (commandes devant passer devant les autres) '''
    self.__mainStack.addLiFo(buff, flag)


  def __sendData(self, buff: str):
    ''' Envoie des donnees sur le port serie '''
    # Signal debug pour toutes les donnees envoyees
    if buff[-2:] == "\r\n":
      self.sig_debug.emit(">>> " + buff[:-2] + "\\r\\n")
    elif buff[-1:] == "\n":
      self.sig_debug.emit(">>> " + buff[:-1] + "\\n")
    else:
      if buff == REAL_TIME_SOFT_RESET:
        self.sig_debug.emit(">>> REAL_TIME_SOFT_RESET")
      elif buff == REAL_TIME_JOG_CANCEL:
        self.sig_debug.emit(">>> REAL_TIME_JOG_CANCEL")
      else:
        self.sig_debug.emit(">>> " + buff)
    # Force l'etat "Home" car grbl bloque la commande ? pendant le Homing
    if buff[0:2] == CMD_GRBL_RUN_HOME_CYCLE:
      self.__decode.set_etatMachine(GRBL_STATUS_HOME)
      self.__grblStatus = GRBL_STATUS_HOME
    # Force l'etat RUN en cas de Probe pour éviter le téléscopage avec les réponses de $#
    if "G38" in buff:
      self.__grblStatus = GRBL_STATUS_RUN
    # Formatage du buffer a envoyer
    buffWrite = bytes(buff, sys.getdefaultencoding())
    # Temps necessaire pour la com (millisecondes), arrondi a l'entier superieur
    tempNecessaire = ceil(1000 * len(buffWrite) * 8 / self.__baudRate)
    timeout = 10 + (2 * tempNecessaire) # 2 fois le temps necessaire + 10 millisecondes
    self.__comPort.write_timeout = timeout
    # Ecriture sur le port serie
    self.sig_debug.emit("grblComSerial.__sendData(), T = {} : timeout = {}".format(time.time() * 1000, timeout))
    self.sig_activity.emit(True)
    try:
      self.__comPort.write(buffWrite)
    except serial.SerialTimeoutException:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial: Error when sending data: timeout, err# = {}").format(self.__comPort.error()))
    except:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial: Unknown error"))
    else:
      self.sig_debug.emit(self.tr("grblComSerial: Data sent, T = {}".format(time.time() * 1000)))
      self.sig_activity.emit(False)


  def __traileLaLigne(self, l, flag = COM_FLAG_NO_FLAG):
    ''' Emmet les signaux ad-hoc pour toutes les lignes recues  '''
    # Envoi de toutes les lignes dans le debug
    if l[-1:] == "\n":
      self.sig_debug.emit("<<< " + l[:-1] + "\\n")
    elif l[-2:] == "\r\n":
      self.sig_debug.emit("<<< " + l[:-2] + "\\r\\n")
    else:
      self.sig_debug.emit("<<< " + l)
    # Premier decodage pour envoyer le signal ah-hoc
    if l[:5] == "Grbl " and l[-5:] == "help]": # Init string : Grbl 1.1f ['$' for help]
      self.sig_init.emit(l)
    elif l == "ok":                            # Reponses "ok"
      if not flag & COM_FLAG_NO_OK:
        self.sig_ok.emit()
    elif l[:6] == "error:":                    # "error:X" => Renvoie X
      if not flag & COM_FLAG_NO_ERROR:
        errNum = int(l.split(':')[1])
        self.sig_error.emit(errNum)
    elif l[:6] == "ALARM:":                    # "ALARM:X" => Renvoie X
      alarmNum = int(l.split(':')[1])
      self.sig_alarm.emit(alarmNum)
    elif l[:1] == "<" and l[-1:] == ">":       # Real-time Status Reports
      self.__grblStatus = l[1:].split('|')[0]
      self.sig_status.emit(l)
    elif l[:5] == "[PRB:": # Probe result
      self.sig_data.emit(l)
      self.sig_probe.emit(l)
    elif l[:1] == "$" or l[:5] == "[VER:" or l[:5] == "[AXS:" or l[:5] == "[OPT:": # Setting output
      self.sig_config.emit(l)
    else:
      self.sig_data.emit(l)


  def __openComPort(self):
    ''' Ouverture du port serie et attente de la chaine d'initialisation en provenence de Grbl '''

    self.sig_debug.emit("grblComSerial.__openComPort(self)")

    openReceiveTimeout = 2000 # Timeout for first Grbl serial message
    openResetTime = 2000      # Time for sending soft reset if init string is not receive from Grbl
    openMaxTime =   5000      # (ms) Timeout pour recevoir la reponse de Grbl apres ouverture du port = 5 secondes

    # Configuration du port serie
    self.__comPort = serial.Serial()
    com_settings = {
      'baudrate':           self.__baudRate,
      'bytesize':           serial.EIGHTBITS,
      'parity':             serial.PARITY_NONE,
      'stopbits':           serial.STOPBITS_ONE,
      'xonxoff':            False,
      'dsrdtr':             False,
      'rtscts':             False,
      'timeout':            SERIAL_READ_TIMEOUT,
      'write_timeout':      None,
      'inter_byte_timeout': None
      }
    self.__comPort.apply_settings(com_settings)

    self.__comPort.port = self.__portName

    # Ouverture du port
    RC = False
    try:
      self.__comPort.open()
    except serial.SerialException as err:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.__openComPort(): Error opening serial port : {0}").format(err))
      self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): Error opening serial port : {0}").format(err))
      self.sig_connect.emit(False)
      return False
    except ValueError as err: #– Will be raised when parameter are out of range e.g. baud rate, data bits.
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.__openComPort(): Parameter out of range : {0}").format(err))
      self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): Parameter out of range : {0}").format(err))
      self.sig_connect.emit(False)
      return False
    except:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.__openComPort(): Unexpected error : {}").format(sys.exc_info()[0]))
      self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): Unexpected error : {}").format(sys.exc_info()[0]))
      self.sig_connect.emit(False)
      return False

    # Ouverture du port OK
    self.sig_connect.emit(True)
    self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial.__openComPort(): comPort {} open.").format(self.__comPort.port))
    self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): comPort {} open.").format(self.__comPort.port))

    # Initialisation Grbl
    tDebut=time.time() * 1000
    self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): Wait for Grbl init... T = {:0.0f} ms...").format(tDebut))

    # Reveille grbl
    self.__comPort.write(("\r\n\r\n").encode('utf-8'))

    # Attente que Grbl emette sur le port série
    while self.__comPort.in_waiting == 0:
      time.sleep(0.01)
      now = time.time() * 1000
      if now > tDebut + openReceiveTimeout:
        self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.__openComPort(): timeout! No reply from Grbl."))
        self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): timeout! No reply from Grbl."))
        self.sig_connect.emit(False)
        return False

    # Attente chaine d'initialisatoin de Grbl
    tReset = False
    tDebut = time.time() * 1000
    while True:
      while self.__comPort.in_waiting:
        try:
          buff = self.__comPort.readline()
        except serial.SerialException as err:
          self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.__openComPort(): Read error: {}".format(err)))
          self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): Read error: {}".format(err)))
          self.sig_connect.emit(False)
          return False
        try:
          l = buff.decode().strip()
          self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): line received: \"") + l + "\"")
          if l[:5] == "Grbl " and l[-5:] == "help]": # Init string : Grbl V.Mx ['$' for help]
            self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): Grbl init string received in {:0.0f} ms, OK.").format(time.time()*1000 - tDebut))
            self.sig_init.emit(l)
            self.__initOK = True
          else:
            self.sig_data.emit(l)
        except UnicodeDecodeError:
          # Trace l'erreur et ignore...
          self.sig_log.emit(logSeverity.warning.value, self.tr("grblComSerial.__openComPort(): utf-8 decode error, buff={}".format(buff)))
      if not self.__initOK:
        now = time.time() * 1000
        if now > tDebut + (openResetTime) and not tReset:
          # Try to send Reset to Grbl at half time of timeout
          self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): No response from Grbl after {:0.0f}ms, sending soft reset...").format(openResetTime))
          self.__sendData(REAL_TIME_SOFT_RESET)
          self.__sendData("\r\n")
          tReset = True
        if now > tDebut + openMaxTime:
          self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.__openComPort(): Grbl initialization: Timeout!"))
          self.sig_debug.emit(self.tr("grblComSerial.__openComPort(): openMaxTime ({}ms) timeout elapsed !").format(openMaxTime))
          self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.__openComPort(): Grbl's init string not received or unknown Grbl version."))
          self.sig_init.emit("Grbl ??? ['$' for help]")
          self.__initOK = True
          return True # On a pas recu la chaine d'initialisation de Grbl mais on essaie quand même...
      else: # self.__initOK
        # Appel de CMD_GRBL_GET_BUILD_INFO pour que l'interface recupere le nombre d'axes et leurs noms
        self.__sendData(CMD_GRBL_GET_BUILD_INFO + "\n")
        return True # On a bien recu la chaine d'initialisation de Grbl

  def __mainLoop(self):
    ''' Boucle principale du composant : lectures / ecritures sur le port serie '''
    flag = COM_FLAG_NO_FLAG
    while True:
      # On commence par vider la file d'attente des commandes temps reel
      while not self.__realTimeStack.isEmpty():
        toSend, flag = self.__realTimeStack.pop()
        self.__sendData(toSend)
      if self.__okToSendGCode == True:
        # Envoi d'une ligne gcode si en attente
        if not self.__mainStack.isEmpty():
          # La pile n'est pas vide, on envoi la prochaine commande recuperee dans la pile GCode
          toSend, flag = self.__mainStack.pop()
          if toSend[-1:] != '\n':
            toSend += '\n'
          if not flag & COM_FLAG_NO_OK:
            if toSend[-2:] == '\r\n':
              self.sig_emit.emit(toSend[:-2])
            else:
              self.sig_emit.emit(toSend[:-1])
          self.__sendData(toSend)
          self.__okToSendGCode = False # On enverra plus de commande tant que l'on aura pas recu l'accuse de reception.
          self.sig_serialLock.emit(self.__okToSendGCode)
      else: # self.__okToSendGCode is False
        self.sig_debug.emit(self.tr("grblComSerial.__mainLoop(): Not OK to send GCode ({}).").format(self.__mainStack.next()))
        # Process events to receive signals;
        QCoreApplication.processEvents()
      #-----------------------------------------------------------------
      # Lecture du port serie
      #-----------------------------------------------------------------
      while self.__comPort.in_waiting:
        # Début d'activité de lecture
        self.sig_activity.emit(True)
        try:
          # Lecture d'une ligne envoyée par Grbl
          buff = self.__comPort.readline()
          l = buff.decode().strip()
          # Fin de lecture
          self.sig_activity.emit(False)
          if l.find('ok') >= 0 or l.find('error') >= 0 or l.find('ALARM') >= 0:
            self.__okToSendGCode = True # Accuse de reception, erreur ou ALARME de la derniere commande GCode envoyee
            self.sig_serialLock.emit(self.__okToSendGCode)
          if l.find('ok') >= 0:
            self.sig_debug.emit(self.tr("grblComSerial: __mainLoop(): ok received"))
          if l.find('error') >= 0:
            self.sig_debug.emit(self.tr("grblComSerial: __mainLoop(): error Grbl received [{}].").format(l))
          if l.find('ALARM') >= 0:
            self.sig_debug.emit(self.tr("grblComSerial: __mainLoop(): ALARM Grbl received [{}].").format(l))
          if l !='':
            self.__traileLaLigne(l, flag)
        except serial.SerialTimeoutException:
          self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.__mainLoop: Timeout when reading serial port!"))
        except serial.SerialException:
          self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.__mainLoop(): Unexpected exception when reading serial port!"))
        except UnicodeDecodeError:
          # Trace l'erreur et ignore...
          self.sig_log.emit(logSeverity.warning.value, self.tr("grblComSerial.__mainLoop(): utf-8 decode error, buff={}".format(buff)))

        # Process events to receive signals;
        QCoreApplication.processEvents()

      if self.__abort:
        self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial.__mainLoop(): Abort received, closing the thread..."))
        break # Sortie de la boucle principale

      # Pooling : Interrogations de Grbl a interval regulier selon la sequence definie par self.__querySequence
      if self.__pooling:
        if (time.time() - self.__lastQueryTime) * 1000 > GRBL_QUERY_DELAY and self.__initOK:
          if len(self.__querySequence[self.__queryCounter]) == 1:
            self.realTimePush(self.__querySequence[self.__queryCounter])
          else:
            if self.__grblStatus == GRBL_STATUS_IDLE:
              self.gcodeInsert(self.__querySequence[self.__queryCounter], COM_FLAG_NO_OK | COM_FLAG_NO_ERROR)
          self.__lastQueryTime    = time.time()
          self.__queryCounter += 1
          if self.__queryCounter >= len(self.__querySequence):
            self.__queryCounter = 0

    # On est sorti de la boucle principale : fermeture du port.
    self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial.__mainLoop(): Closing serial port."))
    self.sig_connect.emit(False)
    self.__comPort.close()
    self.__initOK = False
    # Emission du signal de fin
    self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial.__mainLoop(): End."))


  @pyqtSlot()
  def run(self):
    ''' Demarre la communication avec le port serie dans un thread separe '''

    thread_name = QThread.currentThread().objectName()
    thread_id = int(QThread.currentThreadId())  # cast to int() is necessary
    self.sig_log.emit(logSeverity.info.value, self.tr('grblComSerial.run(): Running "{}" from thread #{}.').format(thread_name, hex(thread_id)))

    if self.__openComPort():
      self.__mainLoop()
    else:
      self.sig_log.emit(logSeverity.error.value, self.tr("grblComSerial.run(): Unable to open serial port!"))
      # Emission du signal de fin
      self.sig_log.emit(logSeverity.info.value, self.tr("grblComSerial.run(): End."))
