# ArduinoJson - arduinojson.org
# Copyright Benoit Blanchon 2014-2020
# MIT License

add_executable(MiscTests
        arithmeticCompare.cpp
        conflicts.cpp
        FloatParts.cpp
        JsonString.cpp
        Readers.cpp
        StringAdapters.cpp
        StringWriter.cpp
        TypeTraits.cpp
        unsigned_char.cpp
        Utf16.cpp
        Utf8.cpp
        version.cpp
        )

set_target_properties(MiscTests PROPERTIES UNITY_BUILD OFF)

add_test(Misc MiscTests)
