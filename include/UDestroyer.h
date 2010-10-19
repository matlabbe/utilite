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

#ifndef UDESTROYER_H
#define UDESTROYER_H

#include "UtiLiteExp.h" // DLL export/import defines

/**
 * This class is used to delete a dynamicaly created 
 * Singleton. Created on the stack of a Singleton, when the 
 * application is finished, his destructor make sure that the 
 * Singleton is deleted.
 */
template <class DOOMED>
class UTILITE_EXP UDestroyer 
{
public:
    UDestroyer(DOOMED* doomed = 0)  : doomed_(doomed) {}
    
    ~UDestroyer()
    {
        if(doomed_)
        {
            delete doomed_;
            doomed_ = 0;
        }
    }

    void setDoomed(DOOMED* doomed)
    {
        doomed_ = doomed;
    }

private:
    // Prevent users from making copies of a 
    // Destroyer to avoid double deletion:
    UDestroyer(const UDestroyer<DOOMED>&);
    void operator=(const UDestroyer<DOOMED>&);

private:
    DOOMED* doomed_;
};

#endif // UDESTROYER_H
