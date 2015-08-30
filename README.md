Cubez
======

Cubez is a 3d physics library written in the [Go][golang] programming language. It is
very largely inspired by Ian Millington's [cyclone-physics][cyclone] and his book
"Game Physics Engine Design".

Currently, this is about as bare bones as it can get and should definitely be
considered early alpha. All APIs should be considered unstable.

Installation
------------

The library can be installed with the following command:

```bash
go get github.com/tbogdala/cubez
```

To build the examples, run the following in a shell:

```bash
cd examples
./build.sh
```


Current Features
----------------



Documentation
-------------

Currently, you'll have to use godoc to read the API documentation and check
out the unit tests to figure out how to use the library.


LICENSE
=======

Cubez is released under the BSD license. See the [LICENSE][license-link] file for more details.


[golang]: https://golang.org/
[license-link]: https://raw.githubusercontent.com/tbogdala/cubez/master/LICENSE
[cyclone]: https://github.com/idmillington/cyclone-physics
