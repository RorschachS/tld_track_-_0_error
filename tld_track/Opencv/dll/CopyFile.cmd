echo off

if "" == "%1" goto End
copy /y %~sdp0*.dll %1*.dll


:End