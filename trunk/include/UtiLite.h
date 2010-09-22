/**
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef UTIL_H_
#define UTIL_H_

/**
 * @namespace Util
 * @brief The Util library namespace.
 * This library contains some utilities for
 * multi-threading, managing events between threads,
 * a logger and useful functions with files.
 * @see StateThread
 * @see EventsManager
 * @see Logger
 * @see File
 */

/** \page util1 Utilities
  *
  * \section intro Introduction
  * La biblioth�que Util peut �tre utilis�e pour la gestion des communications entre les fils,
  * pour l'utilisation des fils portables et l'enregistreur automatique (Logger).  Les trois classes principales
  * sont l'EventsManager, le StateThread et le Logger.
  *
  * \section eventmanager Gestionnaire d'�v�nements
  * L'EventsManager est utilis� pour g�rer la communication entre plusieurs fils dans une application avec
  * l'aide d'�v�nements. L'EventsManager est bas� sur le patron de conception Mediator. Il fait donc
  * l'interm�diaire entre les �v�nements envoy�s par les fils de l'application. Il transfert ensuite les
  * �v�nements � tous ses receveurs (EventsHandler). Les receveurs r�agissent seulement aux �v�nements qu'ils
  * connaissent et ils ignorent les autres. Ce choix de conception permet une plus grande extensibilit� du
  * code car les �metteurs n'ont pas besoin de conna�tre les receveurs et vice-versa. Des nouveaux receveurs
  * peuvent �tre ajout�s sans modifier le code original. L'EventsManager peut �tre vu comme un patron de
  * conception � lui seul.
  *
  * \section statethread Fil d'�tat (StateThread)
  * Un StateThread est un fil am�lior� permettant une utilisation plus s�curitaire de ce dernier.
  * Comme le nom l'indique, c'est un fil avec des �tats (IDLE, RUNNING, KILLED). Le comportement principal
  * que g�re cette classe est l'arr�t s�curitaire du fil en s'assurant d'�tre dans le bon �tat. Elle ajoute
  * un niveau d'abstraction �vitant de manuellement s'assurer que le fil a termin� sa boucle principale pour
  * �tre arr�t�. Un arr�t au milieu de la boucle principale peut causer des interblocages (deadlocks) car le
  * fil peut avoir bloqu� l'acc�s � des donn�es partag�es. Lorsqu'on arr�te un StateThread, il termine sa boucle
  * principale et ensuite s'arr�te automatiquement. Le code des fils de bases portables a �t� r�cup�r� de la
  * version de Phillip Sitbon.
  *
  * \section logger Logger
  * Un Logger est un outil de d�veloppement utilis� pour faire l'enregistrement automatique des messages
  * cr��s par les d�veloppeurs pour conna�tre l'�volution de l'ex�cution de l'application. Utilis�
  * principalement pour d�boguer, le Logger peut faire afficher les messages sur la console
  * (comme un printf ou un cout) ou bien de les �crire dans un fichier. Un niveau (log level) peut �tre
  * ajout�, il permet de faire la s�lection des messages qui seront �crits dans le log.
  *
  */

// TODO Prefix library include files with "U"

#include "UStl.h"
#include "UConversion.h"
#include "UDirectory.h"
#include "UFile.h"
#include "ULogger.h"
#include "UEventsManager.h"
#include "UEventsHandler.h"
#include "UEvent.h"
#include "UProcessInfo.h"
#include "UMutex.h"
#include "USemaphore.h"
#include "UStateThread.h"
#include "UTimer.h"
#include "UMathFunctions.h"
#include "UVariant.h"

#endif /* UTIL_H_ */
