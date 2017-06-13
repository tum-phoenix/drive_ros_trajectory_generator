set(SOURCES
    "src/phoenix_CC2016_service.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/phoenix_CC2016_service/phoenix_CC2016_service.h"
)

include_directories(include)
add_library(phoenix_CC2016_service SHARED ${SOURCES} ${HEADERS})
target_link_libraries(phoenix_CC2016_service PRIVATE lmscore)
