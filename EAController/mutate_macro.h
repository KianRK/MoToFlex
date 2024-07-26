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

#undef MUTATE_ALL

#define MUTATE_ALL 	POS_MUTATE(sc); \
					POS_MUTATE(soft_cfm); \
					POS_MUTATE(cfm); \
					POS_MUTATE(p); \
					POS_MUTATE(i); \
					POS_MUTATE(d); \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						POS_MUTATE(jointFriction[i]); \
					POS_MUTATE(dynamicFriction); \
					POS_MUTATE(flexibleFriction); \
					for (int i=0; i<NUM_OF_GEARTYPES; i++)\
						POS_MUTATE(B[i]); \
					for (int i=0; i<NUM_OF_GEARTYPES; i++)\
						POS_MUTATE(tolerance[i]); \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						POS_MUTATE(mass[i]); \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						POS_MUTATE(gearSc[i]); \
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						POS_MUTATE(length[i]);	\
					for (int i=0; i<NUM_OF_GEARTYPES; i++) \
						POS_MUTATE(innerDamping[i]); 
					

