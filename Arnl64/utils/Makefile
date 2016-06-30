
CFILEEXT=cpp

ifndef CXX
CXX=g++
endif

ifndef CC
CC=gcc
endif


CXXFLAGS+=-g -Wall -D_REENTRANT -fno-exceptions -fPIC
CXXINC=-I../include -I../include/Aria -I../include/ArNetworking  -I../include/ArVideo -I../include/ArVideoEx -I../include/ArVideoOmni
CXXLINK=-L../lib -lBaseArnl -lArNetworkingForArnl -lAriaForArnl -lpthread -ldl -lrt -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic 
CXXSTATICLINK=

LIBTARGETS=../lib/libBaseArnl.so ../lib/libArNetworkingForArnl.so ../lib/libAriaForArnl.so 

ifeq (../include/ArLocalizationTask.h,$(wildcard ../include/ArLocalizationTask.h))
ARNL=
CXXLINK+=-lArnl
CXXSTATICLINK+=-Xlinker -Bstatic -lArnl -Xlinker -Bdynamic 
CXXINC+=-DARNL
LIBTARGETS+=../lib/libArnl.so
endif

ifeq (../include/ArSonarLocalizationTask.h,$(wildcard ../include/ArSonarLocalizationTask.h))
SONARNL=
CXXLINK+=-lSonArnl
CXXSTATICLINK+=-Xlinker -Bstatic -lSonArnl -Xlinker -Bdynamic 
CXXINC+=-DSONARNL
LIBTARGETS+=../lib/libSonArnl.so
endif

ifeq (../include/ArGPSLocalizationTask.h,$(wildcard ../include/ArGPSLocalizationTask.h))
MOGS=
CXXLINK+=-lMogs
CXXSTATICLINK+=-Xlinker -Bstatic -lMogs -Xlinker -Bdynamic 
CXXINC+=-DMOGS
LIBTARGETS+=../lib/libMogs.so
endif

CXXSTATICLINK+=-L../lib -Xlinker -Bstatic -lBaseArnl -lArNetworkingForArnl -lAriaForArnl -Xlinker -Bdynamic -lpthread -ldl -lrt -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic

LIBTARGETS_STATIC:=$(patsubst %.so,%.a,$(LIBTARGETS))


all: makeParams

.PHONY: libs checkCompiler all clean  info depAll cleanAll  src

clean:
	rm -f makeParams makeParamsStatic 

dep: clean 
	if [ -f `echo *.cpp | cut -d' ' -f1` ]; then \
	$(CXX) $(CXXFLAGS) $(CXXINC) -MM *.cpp | \
	awk '$$1 ~ /:/{printf "%s\n", $$0} $$1 !~ /:/' | \
	sed 's/\.o//' > Makefile.dep; fi


Makefile.dep:
	$(MAKE) dep

makeParams: makeParams.$(CFILEEXT) $(LIBTARGETS) Makefile.dep 
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXLINK)

makeParamsStatic: makeParams.$(CFILEEXT) $(LIBTARGETS_STATIC) Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXSTATICLINK)


depAll:
	$(MAKE) -C ../src depAll

cleanAll:
	$(MAKE) -C ../src cleanAll

../lib/lib%: FORCE
	$(MAKE) -C ../src $@

../include/Aria/%.h: ../include/Aria

../include/Aria/Aria.h: ../include/Aria

../include/ArNetworking/%.h: ../include/ArNetworking

../include/ArNetworking/ArNetworking.h: ../include/ArNetworking

../include/ArVideoOmni/%.h: ../include/ArVideoOmni

../include/ArVideoEx/%.h: ../include/ArVideoEx

../include/ArVideoEx/ArLightAnalyzer.h: ../include/ArVideoEx

../include/ArVideo/%.h: ../include/ArVideo

../include/ArVideo/ArVideoInterface.h: ../include/ArVideo

../include/ArSimpleMapping/%.h: ../include/ArSimpleMapping

../include/Aria:
	cd ../include; if test \! -d Aria; then ln -v -s -f $(ARIA)/include Aria; fi

../include/ArNetworking:
	cd ../include; if test \! -d ArNetworking; then ln -v -s -f $(ARIA)/ArNetworking/include ArNetworking; fi

../include/ArVideo:
	cd ../include; if test \! -d ArVideo; then ln -v -s -f $(ARIA)/ArVideo/include ArVideo; fi

../include/ArVideoOmni:
	cd ../include; if test \! -d ArVideoOmni; then ln -v -s -f $(ARIA)/ArVideoOmni/include ArVideoOmni; fi

../include/ArVideoEx:
	cd ../include; if test \! -d ArVideoEx; then ln -v -s -f $(ARIA)/ArVideoEx/include ArVideoEx; fi

../include/ArSimpleMapping:
	cd ../include; if test \! -d ArSimpleMapping; then ln -v -s -f $(ARIA)/ArSimpleMapping/include ArSimpleMapping; fi



FORCE: 

# Tell make not to delete the library targets after building them 
.SECONDARY: $(LIBTARGETS) $(LIBTARGETS_STATIC)

# Cancel make's default rule, which would be used if libraries aren't built yet
%: %.$(CFILEEXT)



ifeq (Makefile.dep,$(wildcard Makefile.dep))
include Makefile.dep
endif


