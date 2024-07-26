/*
	Copyright 2011, Oliver Urbann
	All rights reserved.

	This file is part of MoToFlex.

    MoToFlex is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MoToFlex is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.

	Contact e-mail: oliver.urbann@tu-dortmund.de
*/

#undef COPY_ALL
#undef COPY_RANGE

#define COPY_RANGE(parm, len)

#define COPY_ALL	COPY(sc,); \
					COPY(L,); \
					COPY(R,); \
					COPY(K,); \
					COPY(mu,);\
					COPY(soft_erp,); \
					COPY(soft_cfm,);	\
					COPY(erp,); \
					COPY(cfm,); \
					COPY(p,); \
					COPY(i,); \
					COPY(d,); \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						{COPY(jointFriction,[i]);} \
					COPY(dynamicFriction,); \
					COPY(flexibleFriction,);\
					COPY(fMax,);	\
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						{COPY(B,[i]);} \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						{COPY(tolerance,[i]);} \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						{COPY(mass,[i]);} \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						{COPY(gearSc,[i]);} \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						{COPY(length,[i]);} \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						{COPY(innerDamping,[i]);}
					

