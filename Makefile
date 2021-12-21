.SUFFIXES: .c .cpp .o .mak
.PHONY: FORCE

CC = gcc
CPP = g++
#LD = gcc
LD = g++


C_MOTION = ./C-Motion

C_MOTION_C = $(C_MOTION)/C
C_MOTION_INC = $(C_MOTION)/Include

# Tell make where to look for C source files.
vpath %.c $(C_MOTION_C) ./C
#vpath %.cpp ./C


#OPT = -O2
#CFLAGS = -g $(OPT) -I $(C_MOTION_INC) $(CDEFS)
CFLAGS = -I $(C_MOTION_INC)
CPPFLAGS = -I $(C_MOTION_INC) -I ../ -I ./Source -I ./ -std=c++11
LDFLAGS = -g -static-libgcc 
#-static-libstdc++

obj/%.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@

obj/%.o : %.cpp
	$(CPP) -c $(CPPFLAGS) $< -o $@

obj/%.mak : %.c
	$(CC) -MM $(CFLAGS) $< > $@


c_motion_src = C-Motion.c \
       	       PMDtrans.c \
 		PMDRaspianSer.c \
		PMDdiag.c \
		PMDutil.c \
		Examples.c \
		PMDcommon.c \
	    PMDpar.c
	    
c_motion_obj = $(c_motion_src:%.c=obj/%.o)


## create a cpp_src variable is you need are compiling c++
#cpp_obj = $(cpp_src:%.cpp=obj/%.o)

c_motion_dep = $(c_motion_src:%.c=obj/%.mak)

	
PMDApp: obj $(c_motion_obj) obj/PMDApp.o
	$(LD) $(LDFLAGS) -o $@ obj/PMDApp.o $(c_motion_obj) 
		
PMDPC104: obj $(c_motion_obj) obj/PC104App.o 
	$(LD) $(LDFLAGS) -o $@ obj/PC104App.o $(c_motion_obj)
	
	#$(cpp_obj)

obj:
	mkdir obj
	mkdir obj/Source

c_motion: obj $(c_motion_obj)

clean:
	-rm -f obj/*.o
	-rm -f obj/*.mak
	-rm -f obj/Source/*.o

-include $(c_motion_dep)

