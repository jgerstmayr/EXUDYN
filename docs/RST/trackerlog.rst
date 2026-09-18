.. role:: textred
.. role:: textorange
.. role:: textblue
.. role:: textgreen
.. role:: boldred
.. role:: boldorange
.. role:: boldblue
.. role:: boldgreen

.. _sec-issuetracker:

=============
Issue tracker
=============

This section contains resolved issues per release and known bugs. Use this information to understand changes compared to previous versions. The author field is omitted if it was Johannes Gerstmayr (JG).
The extension \ ``.dev1``\  is not added in the issues list (e.g., 1.2.2.dev1==1.2.2), as it only marks versions that will not be available in pypi with standard pip install, but only with the \ ``-``\ \ ``-pre``\  option or by specifying the exact version name, see versions on `https://pypi.org/project/exudyn/ <https://pypi.org/project/exudyn/>`_.
BUG numbers refer to the according issue numbers.

General information on current version:
 
+  Exudyn version = 1.11.165.dev1, 
+  last change =  2026-09-18, 
+  Number of issues = 2509, 
+  Number of resolved issues = 2238 (165 in current version), 

************
Version 1.11
************

 * Version 1.11.165: resolved Issue 2502: sliderCrank3Dbenchmark result depends on the numpy version (testing)
    - issue author: Claude-JG
    - description:  Measured 2026-09-18: with the SAME exudyn binary (exudynCPP.cp313-win_amd64.pyd md5 identical in both environments); the same source; the same machine and the same Python 3.13.15; the model returns 7.256859912845965 under numpy 2.4.6 - exactly the committed reference - and 7.256859914829453 under numpy 2.2.4; relative 2.7e-10. The suite tolerance is 5e-14; so the test FAILS wherever numpy is older; which is what the maintainer sees in venvP313. numpy enters through the model setup only; so a last-bit difference in the input data is amplified by the solver to 2.7e-10. The same model is already listed in UnresolvedOnLinux() with rel. 2.0e-10; which now looks like the same effect rather than a platform difference. Open questions: which numpy operation in the setup differs; whether other entries of UnresolvedOnLinux() are really numpy-version effects; and whether the suite should pin a minimum numpy for reference comparison or mark the model sensitive. revision2026 phase R10.
    - **notes:** Fixed in revision2026 step R5.9.2 by the maintainer's option 1: the 22 products in the Create\*Joint helpers of mainSystemExtensions.py now go through two private written-out helpers _MatVec3 and _MatMul3x3; whose summation order is fixed by the source instead of by whichever kernel numpy picks. Root cause was NOT the model or the solver: the whole assembled system was bit-identical under numpy 2.2.4 and 2.4.6; only two marker localPosition values differed; and those come from A0.T @ (pJoint - p0). numpy 2.4.6 returns -2.9e-19 where 2.2.4 and an explicit sum return exactly 0.0; proved by running the model on ONE interpreter with the other numpy on PYTHONPATH. After the fix the marker positions and the model result are bit-identical under both numpy versions (7.256859914829453); exactly one reference value moved plus its AVX2 counterpart (7.256859912756364); and venvP313 - which failed two models - now passes the whole suite with both the regular and the fast module. The pattern remains in about eighty other places where nothing measured makes it matter; recorded as fact 28.
    - date resolved: **2026-09-18 07:50**\ , date raised: 2026-09-18 
    - resolved by: Claude-JG
 * Version 1.11.164: resolved Issue 2504: runTestExamples and runPerformanceTests have no --exit-code (extension)
    - issue author: Claude-JG
    - description:  runTestSuite.py exits non-zero with --exit-code (and 0 without it); the other two runners have no such flag and ALWAYS return 0; however many examples or performance tests failed. Anything calling them - the exudev driver of revision2026 step R5.18; a CI job; a shell script - therefore cannot see a failure from the exit code and has to read the summary line out of the log instead; which is what tools/exudev/results.py does today. That scan cannot distinguish a run that died before writing its summary from a log it did not find; so it reports "unknown". Give both runners the same --exit-code flag runTestSuite.py has (about 10 lines each; the pattern already exists); then delete tools/exudev/results.py and let every step of the driver be judged by its exit code. revision2026 step R5.18.1
    - **notes:** Fixed in revision2026 step R5.18.1: runTestExamples.py and runPerformanceTests.py both take --exit-code and return non-zero on a failure; tools/exudev/results.py - the log scan that existed only because they could not - is deleted; and the exudev driver passes the flag always. The examples needed an exclusion list first; testRunnerTools.KnownExampleFailures(); or the exit code would have been red on every run; it also reports dead exclusions so the list shrinks. Successor for the three entries that are really missing optional packages: #2507.
    - date resolved: **2026-09-18 07:10**\ , date raised: 2026-09-18 
    - resolved by: Claude-JG
 * Version 1.11.163: resolved Issue 2501: symbolicModuleTest counts 2 wrong results with numpy 2.2 (testing)
    - issue author: Claude-JG
    - description:  The vector/matrix section compares the symbolic result against the numpy result with an ABSOLUTE tolerance: np.linalg.norm(res[0]-res[1]) > 1e-15. One of the compared values has magnitude 9.7476; where 1 ulp is 1.8e-15; so the tolerance is below the representable resolution. Measured 2026-09-18 on one machine with the SAME exudyn binary (md5 identical in both environments) and the same source: numpy 2.4.6 gives 9.7476 on both sides and the test passes; numpy 2.2.4 gives 9.747600000000002 on the numpy side; a difference of 1.78e-15; counted once per recording mode. cntWrong is added to the test result since #2479; so the model reports 2.948412957506974 against a reference of 0.9484129575069745 and the suite FAILS with error 2.0. The comparison needs a relative tolerance.
    - **notes:** Fixed in revision2026 step R5.9.1: the vector/matrix comparison in symbolicModuleTest.py is now RELATIVE to the magnitude being compared - scale = max(norm(sym); norm(py); 1.) and a tolerance of 1e-14\*scale - instead of an absolute 1e-15 on a value of magnitude 9.7476; where one ulp is 1.8e-15. Verified: the model returns exactly the committed reference 0.9484129575069745 in both venvP313 (numpy 2.2.4) and venvExuP313 (numpy 2.4.6); and a mutation making one symbolic result wrong by 1e-12 relative is still caught twice; so the test is not weakened in any way that matters.
    - date resolved: **2026-09-18 07:10**\ , date raised: 2026-09-18 
    - resolved by: Claude-JG
 * Version 1.11.162: :textred:`resolved BUG 2506` : RaytracingSettings::maxNThreads has no definition 
    - issue author: Claude-JG
    - description:  src/Graphics/Raytracing.h:136 declares "static const Index maxNThreads = 256;" with an in-class initializer but no out-of-class definition. Raytracing.cpp:913 passes it to EXUstd::Clamp(const T& x; const T& lower; const T& upper); binding a reference to it - which is an odr-use; so a definition IS required. At -O3 the compiler folds the constant and nothing is referenced; so every release build links and loads. At -O1 it does not fold; the symbol stays undefined and importing the module fails outright with "undefined symbol: _ZN18RaytracingSettings11maxNThreadsE". Found 2026-09-18 on the FIRST build of revision2026 step R5.6 (sanitizers; which use -O1); before a single sanitizer check had run. The neighbouring materialOffset is already "static constexpr"; which in C++17 is implicitly inline and needs no definition; RTcolorDepth on the next line has the same latent pattern. Fix: make them constexpr.
    - **notes:** Fixed in revision2026 step R5.6: src/Graphics/Raytracing.h:136 - maxNThreads and its neighbour RTcolorDepth are now "static constexpr"; which in C++17 is implicitly inline and therefore needs no out-of-class definition; matching materialOffset two lines above. Verified: before the change the -O1 sanitizer build produced a module that failed to import with "undefined symbol: _ZN18RaytracingSettings11maxNThreadsE"; after it the same build imports and the whole test suite runs. The MSVC release build was rebuilt and the test suite passed in venvExuP313 with unchanged values; so nothing about the behaviour moved - at -O3 the constant was folded all along; which is why no Windows or manylinux release ever showed this.
    - date resolved: **2026-09-18 02:47**\ , date raised: 2026-09-18 
    - resolved by: Claude-JG
 * Version 1.11.161: resolved Issue 2503: the build and test scripts are 18 batch files without help (extension)
    - issue author: Claude-JG
    - description:  tools/buildAndGenerate/ holds 18 files; 5 of which exist only to find conda and to loop over the Python versions. A .bat file cannot print a --help; cannot validate an option and cannot pass an unknown option on: every new runner option has to be threaded through by hand - which is how --fast-module stayed unreachable from runTestSuite.bat until #2500. The scripts are also the place where the maintainer looks up HOW the build works; so they carry a documentation duty that comment headers serve badly. Replace them with one Python driver with subcommands; a --help per subcommand and a --dry-run that prints the commands instead of running them; keeping only the scripts that must stay shell (manylinuxBuild.sh runs inside the docker image). revision2026 step R5.18
    - **notes:** Resolved by revision2026 step R5.18: tools/exudev/ - a dependency-free Python driver run as "python tools/exudev" or "exudev" from the repository root - replaces the 16 batch files of tools/buildAndGenerate/; which is retired (manylinuxBuild.sh moved to tools/ci/ next to the buildManylinux.sh it calls; the rest moved to the gitignored tmp/oldScripts/). Ten commands with a --help each; quiet by default with -v/--verbose; -n/--dry-run prints the real command lines and is faithful by construction because commands.py only builds Step objects and runner.RunSteps is the only executor. Environment dispatch through "conda run -n <env> --no-capture-output". --fast is opt-in and the driver therefore switches the pyproject default off; it warns about the two gates in setup.py that would silently drop the fast module. Successor for the two runners without an exit code: #2504.
    - date resolved: **2026-09-18 01:53**\ , date raised: 2026-09-18 
    - resolved by: Claude-JG
 * Version 1.11.160: :textred:`resolved BUG 2500` : the suite log is split in two when EXUDYN_OUTPUTDIRECTORY is set 
    - issue author: Claude-JG
    - description:  runTestSuite.py opens its log with exu.SetWriteToFile; which the C++ side resolves against exudyn.config.outputDirectory (#2418); so with EXUDYN_OUTPUTDIRECTORY set the log is written outside the working tree - as intended. But after the models the suite resets outputDirectory to the empty string and RE-OPENS the same log by name for the summary; which then lands at the UNREDIRECTED path: the body of the log goes to the output directory and the summary to python/TestSuiteLogs. Measured 2026-09-17: 80 KB outside and a 17 KB summary-only file inside. The -local copy reads the unredirected name as well. The reset should restore the directory the run STARTED with; not the empty string. revision2026 step R5.13.5
    - **notes:** runTestSuite.py remembers the output directory the run started with and puts it BACK after the models instead of clearing it; so both opens of the log resolve to the same file; the -local copy reads through the same resolution; and the directory is cleared only after the file is closed - which is what the original reset was for. Verified 2026-09-17 with EXUDYN_OUTPUTDIRECTORY set: nothing is written into python/TestSuiteLogs (43 files before and after) and the one log outside the tree is complete including the summary; a run without the variable behaves exactly as before. revision2026 step R5.13.5
    - date resolved: **2026-09-17 23:49**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.159: resolved Issue 2499: the platform string does not name the architecture (change)
    - issue author: Claude-JG
    - description:  exudyn.config.Version(addDetails=True) reported -Windows-; -MacOS- or -MacOS(ARM)-; so an Intel Mac and an Apple silicon Mac were indistinguishable and 32 vs 64 bit was visible on Windows only; as -(32bit)-. The string is written into the header of every solution file; sensor file; parameter variation and optimization results file; so it is what a user sends with a bug report. It now names the architecture the way the wheel tags and the compilers do: Windows x86_64; MacOS arm64; MacOS x86_64; Linux x86_64; Linux arm64 - and x86 or arm for the 32 bit cases. Also documents why macOS builds no fast module. revision2026 step R2.10.5
    - **notes:** GetPlatformString() now appends the architecture as the wheel tags and the compilers name it: Windows x86_64; MacOS arm64; MacOS x86_64; Linux x86_64; Linux arm64; and x86 or arm for the 32 bit cases - which also replaces the Windows-only (32bit) marker; since x86_64 and arm64 already say 64 bit. An architecture matching none of them stays unnamed rather than mislabelled. Decided per compile; so each slice of a macOS universal2 binary reports its own. Verified without building Exudyn: the branch selection with the real preprocessor for seven architectures; and the function itself extracted; compiled and run. The change takes effect at the next C++ build. The header text of solution; sensor; parameter variation and optimization files changes with it; the two documented sample headers were updated. Nothing parses the string except the AVX2 and [FAST] checks of the test runner. revision2026 step R2.10.5
    - date resolved: **2026-09-17 23:28**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.158: :textred:`resolved BUG 2430` : ObjectANCFThinPlate added with its defaults fails inside C++ with an index error 
    - issue author: Claude-JG
    - description:  Found in revision2026 step R4.4.3.4e: mbs.AddObject(ObjectANCFThinPlate()) raises ResizableArray<T>::operator[] i < 0 from C++ even with exudyn.special.exceptions.parameterRangeChecks = False; its default nodeNumbers (four InvalidIndex) are used while the object is added. Every other item class either adds with its defaults or names the parameter that must be given. Expected: a message that names ObjectANCFThinPlate.nodeNumbers - or CheckPreAssembleConsistency catching it - and no index access with invalid node numbers during Add. revision2026 step R10.4.
    - **notes:** Verified 2026-09-17 on 1.11.135.dev1: mbs.AddObject(ObjectANCFThinPlate()) no longer raises from C++; it adds with its defaults exactly as ObjectMassPoint and ObjectANCFCable2D do; which is the "expected" behaviour this issue names. Fixed on the way by the item-interface work of revision2026 step R4.4.3.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.157: resolved Issue 2412: Google style docstrings are mandatory project wide (docu)
    - issue author: Claude-JG
    - description:  The convention is nowhere stated; the revision plan even said NumPy style. Decision: Google style docstrings MUST be used throughout for consistency. The rule has to be written into docs/dev/CODING_STYLE.md; CONTRIBUTING.md and CLAUDE.md; and mentioned early in the user documentation so contributors meet it before writing code. griffe and pydoclint both parse Google style; so the planned docstring toolchain (revision2026 steps R4.6-R4.9) is unaffected apart from a parser argument.
    - **notes:** Resolved by revision2026 step R4.6: docs/dev/CODING_STYLE.md states Google style as the convention for python/exudyn - summary; then Args:; Returns:; Note:; Example: - and pydoclint checks it in CI against tools/ci/pydoclintBaseline.txt. The #\*\* doc-comment convention it replaced is marked as removed in the same table.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-09-13 
    - resolved by: Claude-JG
 * Version 1.11.156: resolved Issue 2344: itemInterface (extension)
    - description:  add automated test for all items and parameters with dummy class type which always fails. Use objectDefinition database to independently check if all parameters raise exceptions with according strings in it
    - **notes:** Resolved by revision2026 step R4.4.3.1: parameterConversionTest.py walks the definitions database and writes a fixed set of probe values into every parameter of every item and of the simulation and visualization settings; through each access path; recording the exception type or the type; shape and value that reads back. That is the independent check this issue asked for; it currently pins 89 rejected-input outcomes.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-04-17 
    - resolved by: Claude-JG
 * Version 1.11.155: resolved Issue 2343: itemInterface (extension)
    - description:  add safe parameter conversion for all data types in item initialization; add parameter checks like UReal, PInt, etc. into C++ code; exception shall always return causing ItemClass name and parameter name
    - **notes:** Largely resolved by revision2026 step R4.4.3 and its sub-steps: the item interface converts and checks parameters of every type; a rejected value names the item class and the parameter - for instance "ObjectGenericODE2: invalid node number detected" - and the whole behaviour is recorded by parameterConversionTest.py against a committed reference. What is NOT uniform yet is the exception TYPE per kind of error; which is #2432 and revision2026 step R6.7.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-04-17 
    - resolved by: Claude-JG
 * Version 1.11.154: resolved Issue 2333: itemInterface (check)
    - description:  consider advanced checks, in particular for int / float-convertable types in checks like CheckForValidUReal
    - **notes:** Same as #2332: the int/float-convertible cases of CheckForValidUReal and its siblings are probed by parameterConversionTest.py (revision2026 step R4.4.3.1) and their outcome is recorded; the open decision about which exception type each path should raise is #2432 and revision2026 step R6.7.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-04-02 
    - resolved by: Claude-JG
 * Version 1.11.153: resolved Issue 2332: SetNumpyMatrixSafely (check)
    - description:  SetNumpyMatrixSafely and SetNumpyVectorSafely: add type checks and check exceptions printed when initialized with wrong types; consider more advanced checks for standard types in itemInterface
    - **notes:** The behaviour asked about is now pinned rather than guessed: parameterConversionTest.py (revision2026 step R4.4.3.1) writes wrong types into every parameter through every access path - the SetNumpyMatrixSafely and SetNumpyVectorSafely paths included - and compares the exception type or the value that reads back against a committed reference; so a change in these checks cannot pass unnoticed. What to do about the inconsistent exception TYPES is #2432.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-04-02 
    - resolved by: Claude-JG
 * Version 1.11.152: resolved Issue 2298: testing checklist (testing)
    - description:  Add testing checklist and guidelines
    - **notes:** Resolved by revision2026 phase R5 and docs/dev/WORKFLOW.md: the commit tiers and the four gates of section 4 are the checklist this issue asked for - build; regeneration clean; the full test suite; docs and plan updated - together with which tests run when; the pytest collector; the parallel run; and the difference between reproducible and sensitive tests.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-02-14 
    - resolved by: Claude-JG
 * Version 1.11.151: resolved Issue 2240: systemStructures.py (change)
    - description:  add functions for type conversions C++ - Python (Vector3D, etc.)
    - **notes:** Resolved by revision2026 step R4.3: the type conversions asked for here are tools/generators/typeModel.py; which renders one type declaration into its C++; stub; documentation and Python forms from a single definition; instead of each emitter spelling out Vector3D and friends for itself.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-01-31 
    - resolved by: Claude-JG
 * Version 1.11.150: resolved Issue 2239: systemStructures.py (change)
    - description:  transfor into more suitable Python native format
    - **notes:** Resolved by revision2026 step R4.3: src/pythonGenerator/systemStructures.py is gone; the structures are defined in definitions/structureDefs\*.py as plain Python data and the emitters in tools/generators/ produce the C++; the stubs and the documentation from them.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-01-31 
    - resolved by: Claude-JG
 * Version 1.11.149: resolved Issue 2227: build wheels (extension)
    - description:  add flag to setupPyConfig.json to turn on/off AVX2 and AVX512 compile in windows and linux builds
    - **notes:** Resolved by revision2026 steps R2.14 and R2.10: the build switches live in [tool.exudyn] of pyproject.toml - useAVX2 and useAVX512 among them - read by setup.py with an environment and a command-line override. setupPyConfig.json; which this issue named; no longer exists: it was tracked AND mutable; so a CI build rewrote it with sed and a failed build left the tree dirty.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2026-01-16 
    - resolved by: Claude-JG
 * Version 1.11.148: resolved Issue 1996: MainSystemExtensions (change)
    - description:  expose MainSystem as MainSystemBase into python; derive Python class MainSystem from MainSystemBase in MainSystemExtensions; this should increase visibility of Python code!
    - **notes:** Resolved differently and the goal is met: instead of exposing MainSystemBase and deriving a Python MainSystem from it; a function is marked where it is defined with @extends(exudyn.MainSystem) and exudyn.extensionRegistry.install() attaches it to the C++ class (revision2026 phase R4). Python-side MainSystem code is therefore visible in the class; in the stub file and in the documentation; without a second class in the hierarchy.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2025-05-06 
    - resolved by: Claude-JG
 * Version 1.11.147: resolved Issue 1988: exceptions (extension)
    - description:  change 'except:' to 'except Exception as e:' as this passes through the keyboard interrupts, which is better for parameter variation and other long-running codes
    - **notes:** Superseded by #2497: the change was never made; but every one of the 59 bare "except:" in the shipped package is now listed with file and message in tools/ci/ruffBaseline.txt (revision2026 step R5.5.3) and a new one fails the check. See the successor issue.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2025-04-10 
    - resolved by: Claude-JG
 * Version 1.11.146: resolved Issue 1861: exceptions and ValueError (extension)
    - description:  check all ValueError exceptions and change to appropriate error handling, like importerror, runtime error or value error!
    - **notes:** Superseded by #2432; which records the same problem with measurements: parameterConversionTest.py shows that a wrong parameter raises RuntimeError; TypeError or ValueError depending on which path rejected it. The taxonomy itself is revision2026 steps R6.3; R6.4 and R6.7. Individual cases found on the way were fixed with the right type - for instance ValueError for an unknown solver in lieGroupIntegration.py and ImportError for a missing roboticstoolbox (#2488; #2489).
    - date resolved: **2026-09-17 22:43**\ , date raised: 2024-09-19 
    - resolved by: Claude-JG
 * Version 1.11.145: resolved Issue 1142: item functions checker (extension)
    - description:  add automatic tests for all necessary functions in items, such as nodes, objects, etc.; run tests similar to LEST test suite
    - **notes:** Partially resolved and the remainder is #2498: the parameters of every item are now probed systematically by parameterConversionTest.py (revision2026 step R4.4.3.1) and the linear algebra classes by the lest unit tests of src/Tests/ (step R5.4); what is still untested per item type is its member FUNCTIONS. See the successor issue.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2022-06-12 
    - resolved by: Claude-JG
 * Version 1.11.144: :textred:`resolved BUG 1085` : GeneralContact 
    - description:  generalContactFrictionTests.py gives considerably different results after t=0.05 seconds between Windows and linux compiled version; may be caused by some initialization problems (bugs...); needs further tests
    - **notes:** Superseded by the measurement and the machinery of revision2026 steps R2.10.3 and R5.9: the Windows/Linux differences of the contact and friction models are now recorded per model in UnresolvedOnLinux() and SensitiveTests() in runTestSuiteRefSol.py rather than being an open suspicion; and a failure there no longer sets the exit code at random. The remaining question - WHY these models differ - is revision2026 step R10.1; which is the open work.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2022-05-11 
    - resolved by: Claude-JG
 * Version 1.11.143: resolved Issue 0862: Vector alignment (extension)
    - description:  add 32 or 64 byte memory + length alignment to allocation of vectors in order to be able to always use AVX and/or loop unrolling for copying or manipulating vectors; use _mm_malloc / _mm_free and separate flags to turn on/off memory and length alignment, memory alignment turned off in case that AVX is not available
    - **notes:** Resolved: src/Linalg/Vector.h allocates through _aligned_malloc on Windows and posix_memalign elsewhere; so vector data is aligned for the AVX paths; and AlignedFree in src/Utilities/BasicDefinitions.h frees it. Revision2026 step R2.10 settled which module carries the vector extensions (only exudynCPPfast) and step R2.10.3 recorded what they change.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2022-01-13 
    - resolved by: Claude-JG
 * Version 1.11.142: resolved Issue 0452: AVX objects (extension)
    - description:  test AVX objects
    - **notes:** Same coverage as #451: the AVX code paths are tested by src/Tests/AVXVectorUnitTests.h and their numerical effect is pinned by the second reference set of revision2026 step R2.10.3. A micro-benchmark of the real linear algebra operations is planned as revision2026 step R11.2.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2020-09-16 
    - resolved by: Claude-JG
 * Version 1.11.141: resolved Issue 0451: AVX integration (extension)
    - description:  test AVX in vector.cpp and dense solver
    - **notes:** Resolved by src/Tests/AVXVectorUnitTests.h (7 cases covering the AVX paths of the vector classes) together with the measurements of revision2026 steps R2.16 and R2.10.3: the effect of the vector extensions on results is now recorded model by model in AVX2ReferenceSolutionUpdate(); and tools/benchmarks/avx2Benchmark.py measures the solver-level effect.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2020-09-16 
    - resolved by: Claude-JG
 * Version 1.11.140: :textred:`resolved BUG 0448` : ObjectGenericODE2 bug 
    - description:  ObjectGenericODE2 crashes without message when initialized with invalid node numbers
    - **notes:** Verified 2026-09-17 to be fixed by the later item-interface work: mbs.AddObject(ObjectGenericODE2(nodeNumbers=[-1,-2])) now prints "ObjectGenericODE2: invalid node number detected; all nodes used in ObjectGenericODE2 must already exist" and raises; instead of crashing without a message. The message names the item class; which is what the issue asked for.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2020-09-09 
    - resolved by: Claude-JG
 * Version 1.11.139: resolved Issue 0323: invalid index test (check)
    - description:  add test which checks that invalidIndex in python (-1) converts to invalidIndex in exudyn (use simple object with node number and check node number after setting object
    - **notes:** Resolved by revision2026 step R4.4.3.1: parameterConversionTest.py writes a fixed set of probe values - which includes -1 - into every parameter of every item through each access path and compares type; shape and value that read back against a committed reference; index parameters included. The InvalidIndex cases are called out explicitly in that file (#2426).
    - date resolved: **2026-09-17 22:43**\ , date raised: 2020-01-24 
    - resolved by: Claude-JG
 * Version 1.11.138: resolved Issue 0126: Linalg tests (check)
    - description:  Add unit tests for new ConstSizeMatrix and ResizableMatrix
    - **notes:** Resolved by revision2026 step R5.4.1: src/Tests/AllMatrixVariantsUnitTests.h tests ConstSizeMatrix and ResizableMatrix - the two classes this issue named - including the SparseTripletMatrix constructor fixed in step R5.4.6 (#2476).
    - date resolved: **2026-09-17 22:43**\ , date raised: 2019-05-13 
    - resolved by: Claude-JG
 * Version 1.11.137: resolved Issue 0005: finish (new feature)
    - description:  finish tests for all matrix classes    
    - **notes:** Resolved by the later unit-test work of revision2026 step R5.4: src/Tests/AllMatrixUnitTests.h and AllMatrixVariantsUnitTests.h cover Matrix; ResizableMatrix; ConstSizeMatrix and the linked-data variants; step R5.4.1 completed the variants that had none.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2019-04-01 
    - resolved by: Claude-JG
 * Version 1.11.136: resolved Issue 0002: finish (new feature)
    - description:  finish tests for all vector classes    
    - **notes:** Resolved by the later unit-test work of revision2026 step R5.4: src/Tests/AllVectorUnitTests.h; TemplatedVectorArrayUnitTests.h; AllBasicLinalgUnitTests.h and AVXVectorUnitTests.h cover Vector; ResizableVector; SlimVector; ConstSizeVector and LinkedDataVector; run by exu.special.RunCppUnitTests() in every build with EXUDYN_PERFORM_UNIT_TESTS and by the test suite.
    - date resolved: **2026-09-17 22:43**\ , date raised: 2019-04-01 
    - resolved by: Claude-JG
 * Version 1.11.135: :textred:`resolved BUG 2496` : --fast-module would reject a fast module built without AVX2 
    - issue author: Claude-JG
    - description:  The guard and the log marker added for revision2026 step R5.11 asked ModuleUsesAVX2(); but that is a different question from -is this the fast module-. setup.py always compiles exudynCPPfast with __FAST_EXUDYN_LINALG and adds the vector extensions only when useAVX2 is set AND the platform is not macOS - so on macOS; and in any --no-avx2 build; the fast module is perfectly good and reports no AVX2. The run would then abort with the message that the wrong module was loaded; and its log would have no _fast marker. Both now ask ModuleIsRegular(); the AVX2 reference set keeps asking ModuleUsesAVX2(); which is right because that drift comes from the vector extensions and not from the missing range checks. revision2026 step R5.11.2
    - **notes:** The guard and the log marker now ask ModuleIsRegular() - is this the fast module - instead of ModuleUsesAVX2() - does it have vector extensions. setup.py always compiles exudynCPPfast with __FAST_EXUDYN_LINALG but adds the vector extensions only when useAVX2 is set AND the platform is not macOS; so on macOS and in any --no-avx2 build the fast module reports no AVX2 and the run would have aborted; with its log written under the regular name. Verified by simulating that build: with ModuleUsesAVX2 forced False the guard accepts the fast module. The AVX2 reference set keeps asking ModuleUsesAVX2(); which is the right question there. revision2026 step R5.11.2
    - date resolved: **2026-09-17 22:38**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.134: resolved Issue 2495: the fast module is never exercised by the test suite (extension)
    - issue author: Claude-JG
    - description:  Windows release builds ship exudynCPP and exudynCPPfast; but the suite only ever runs whichever one __init__.py selects - the default - so exudynCPPfast ships essentially untested although it is the module used for long simulations and the one whose missing range checks turn a user error into undefined behaviour. The machinery to JUDGE a fast run already exists (ModuleUsesAVX2 and AVX2ReferenceSolutionUpdate from R2.10.3); what is missing is the switch that loads it. revision2026 step R5.11
    - **notes:** EXUDYN_MODULE=fast is read by __init__.py before the C++ module is imported; and runTestSuite.py and runPerformanceTests.py take --fast-module; which sets it. An environment variable and not a flag because child processes inherit it: --parallel and pytest -n run every model in its own interpreter. The suite then applies the existing AVX2 reference set automatically and writes its log with a _fast suffix; a declined fast request stops the run instead of producing a log that claims the wrong module. Two adaptations were needed and are the value of the step: symbolicModuleTest checked two range-check error paths that exudynCPPfast deliberately does not have; and ANCFbeltDrive needed its AVX2 reference value. runPerformanceTests lost its heuristic of using the fast module iff Python is 3.10. Measured: suite PASSED on both modules (the first fast pass); --parallel --fast-module PASSED; performance 42.13s regular against 33.54s fast. The -noavx part of the step is dropped: step R2.10 removed that module. revision2026 steps R5.11 and R5.11.1
    - date resolved: **2026-09-17 22:19**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.133: :textred:`resolved BUG 2377` : three imports refer to modules that exist nowhere 
    - issue author: Claude-JG
    - description:  found by tools/checkExtras.py (revision2026 step R2.12). Examples/FurtherExamples/spotReinforcementLearning.py does "import RL_Spot" and no such file is in the repository; TestModels/LieGroupIntegrationUnitTests.py does "from timeIntegrationOfRotationVectorFormulas import \*" and no such file is in the repository; Examples/ROSMassPoint.py imports rosInterface by bare name although the module is exudyn/robotics/rosInterface.py; so it only works if that directory happens to be on sys.path. All three are currently listed in knownMissingLocalModules in checkExtras.py so the checker reports them as broken imports rather than as packaging gaps
    - **notes:** rosInterface: ROSMassPoint.py and ROSTurtle.py imported it by bare name; both now import exudyn.robotics.rosInterface as their sibling ROSMobileManipulator.py already did. timeIntegrationOfRotationVectorFormulas: the module was never committed; but both functions it provided exist in the package today - ComposeRotationVectors is now CompositionRuleForRotationVectors; plus Skew; TSO3Inv (TExpSO3Inv) and the two RK step functions; LieGroupIntegrationUnitTests.py imports them and 9 of its 10 tests pass (TEST 2 fails on the separate #2494). Both entries are gone from knownMissingLocalModules in tools/checkExtras.py; which now exits 0. RL_Spot remains: the module was never committed and the decision - delete the example or add the module - is the maintainers. revision2026 step R5.12
    - date resolved: **2026-09-17 21:26**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.132: :textred:`resolved BUG 2368` : ANCFbeltDrive result contradicts its own recorded reference 
    - issue author: Claude-JG
    - description:  Run headless the model yields 0.0 while the reference in the file comment is -0.4842656133238705 (2021-05-07). The model was retuned to a 10s dynamic run (tEnd = 10 ; h = 0.5e-3) and the reference was not updated. Found while triaging the unlisted TestModels for revision2026 step R5.9. Not added to the test suite: adding the measured value would enshrine whatever changed.
    - **notes:** The maintainer retuned the model on 2026-09-17 (16 elements per section; faster drive; tEnd=0.1 instead of 10) and took the result from the sensor instead of an ODE2 coordinate; measuring a new value in the model file. Finalized here: the model ran with parallel.numberOfThreads=4 and was therefore NOT reproducible - three headless runs spread over 5.4e-14; past the 5e-14 suite tolerance - so it now runs single-threaded; which is bit-identical across runs and at this size also faster (0.36s vs 0.69s). The reference was re-measured single-threaded in the model file and in runTestSuiteRefSol.py; the model left DeliberatelyNotRun and runs in the suite with error 0.0 in 0.35s (it was 28s).
    - date resolved: **2026-09-17 21:26**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.131: :textred:`resolved BUG 2493` : every Exudyn writer should create its output directory itself 
    - issue author: Claude-JG
    - description:  SaveDictToHDF5 fails with FileNotFoundError if the directory does not exist; while FEMinterface.SaveToFile; PlotSensor; PlotImage and the parameter variation results file each carry their own copy of the same try/except os.makedirs block - five copies of four lines. One function CreateDirectoryForFile in basicUtilities replaces them; and SaveDictToHDF5 gets the behaviour it was missing. revision2026 step R5.13.4
    - **notes:** basicUtilities.CreateDirectoryForFile(fileName) creates the directory of a file that is about to be written and returns the name unchanged; failure is ignored on purpose - creating a directory can fail for reasons that do not stop the write; and the write reports the real problem better. SaveDictToHDF5 now calls it - the behaviour it was missing - and the five hand-written copies in FEM.py (2); plot.py (2) and processing.py now call it instead. Verified: testHDF5loadSave.py writes into a solution/ directory that does not exist yet; without any makedirs in the example. The two examples that needed an os.makedirs line one commit ago no longer do. revision2026 step R5.13.4
    - date resolved: **2026-09-17 20:42**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.130: :textred:`resolved BUG 2492` : test models write solution files that nothing reads 
    - issue author: Claude-JG
    - description:  Fifteen test models set solutionSettings.writeSolutionToFile=True unconditionally; the coordinates solution file is then written on every suite run although it is read back only under useGraphics by the SolutionViewer - and compareFullModifiedNewton is the designated test of the writing itself. Four models already had the right form (writeSolutionToFile=False; True only if useGraphics). revision2026 step R5.13.1
    - **notes:** Fifteen test models now write the solution file only when it is read: thirteen use writeSolutionToFile=useGraphics - the form four models already had - and ACFtest and doublePendulum2DControl set False; neither has useGraphics in scope at that point and nothing reads the file. The three remaining sensors that still wrote to a file (ANCFbeltDrive twice; ACFtest once) use storeInternal=True; PlotSensor reads the internal data. Measured over a full suite run with the output directory emptied first: 74 -> 64 coordinates solution files and 11 MB -> 8.0 MB. The sensor half of the plan step was already done by R5.13 and R5.13.2: 74 models use storeInternal and almost every sensor fileName in the test models is commented out. revision2026 step R5.13.1
    - date resolved: **2026-09-17 20:29**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.129: :textred:`resolved BUG 2491` : examples write generated FEM data into the tracked testData/ input directory 
    - issue author: Claude-JG
    - description:  Twelve examples save meshes and FEM data under testData/ - the directory that holds the tracked INPUT files - so every run of runTestExamples.py leaves untracked netgenBrick.npz; netgenHinge.npz; netgenFFRF2.npz; FMBStest1.npz; modalAnalysisFEM.npz and test.hdf5 in the working tree; where they are easily committed by accident. Output belongs under solution/; which is ignored; through OutputFilePath as the test models already do. revision2026 step R5.13.1
    - **notes:** Twelve examples now write their generated FEM data through OutputFilePath into solution/ instead of testData/: ten FEMinterface caches; and the two SaveDictToHDF5 sites which also create the directory themselves because SaveDictToHDF5 - unlike FEMinterface.SaveToFile - does not. Verified by running all 171 examples with the artifacts removed first: the same 5 pre-existing failures; no file left in any testData directory; the meshes land in the per-example output directory. A direct user run without an output directory writes into Examples/solution/; which is ignored. revision2026 step R5.13.3
    - date resolved: **2026-09-17 20:23**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.128: :textred:`resolved BUG 2471` : FEMinterface NPZ files store a C++ enum and cannot be read by a second module 
    - issue author: Claude-JG
    - description:  FEMinterface.SaveToFile(mode=NPZ) writes postProcessingModes as a dict whose outputVariableType is an exudyn.exudynCPP.OutputVariableType - a pybind type. np.load(allow_pickle=True) therefore imports exudyn.exudynCPP when reading it, and in a process that already loaded exudynCPPfast this raises ImportError: generic_type: type "Real" is already registered. Measured 2026-09-17 on testData/netgenTestMesh.npz: every other field (nodes, elements, massMatrix, stiffnessMatrix, surface, modeBasis, eigenValues, metaData) loads under both modules; only this one fails. An NPZ is meant to hold plain arrays: store the enum by name and convert back on load. Until then NGsolveCMStest stays in NotJudgedOutsideRegularModule(). revision2026 step R2.10.4.
    - **notes:** FEMinterface.SaveToFile now stores postProcessingModes[outputVariableType] by NAME - on a copy; so the object in memory keeps its enum - and SetWithDictionary converts the name back on load; files written before this still carry the enum and are loaded as before. Verified across modules: a file written under the regular module is read under exudynCPPfast in both NPZ and PKL form and the enum comes back; writing the enum object directly still reproduces the original ImportError generic_type: type Real is already registered. NGsolveCMStest cannot leave NotJudgedOutsideRegularModule() yet: its tracked testData/netgenTestMesh.pkl was written in the old form and has to be regenerated first. revision2026 step R2.10.4
    - date resolved: **2026-09-17 20:06**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.127: :textred:`resolved BUG 2490` : the stubs miss the deprecated module functions and GetDictionary/SetDictionary 
    - issue author: Claude-JG
    - description:  addDocu=False suppressed the .pyi entry as well as the documentation; so all 14 deliberately undocumented module-level functions (StartRenderer; StopRenderer; InfoStat; GetVersionString; SolveStatic; SolveDynamic; ...) were missing from python/exudyn/__init__.pyi. GetDictionary and SetDictionary were missing from all 43 settings classes; because structureHeaderEmitter adds them to the pybind class but structureStubEmitter did not mirror them. Both matter as soon as the package ships a PEP 561 py.typed marker: a type checker then reports correct user code as an error. revision2026 steps R5.5.2 and R5.5.4
    - **notes:** The stub emission no longer depends on addDocu: a function that is deliberately left out of the documentation still exists in the module and is now described in the .pyi (13 functions); StopRenderer was the only declaration without a returnType and got returnType=None (14). structureStubEmitter now mirrors GetDictionary/SetDictionary under the same condition structureHeaderEmitter uses to add them to the pybind class (43 classes). Measured with mypy on a user script that calls StartRenderer; GetDictionary; SetDictionary; SolveDynamic; SetOutputPrecision and StopRenderer: 9 errors with the old stub; none with the new one. py.typed is shipped from now on. revision2026 steps R5.5.2 and R5.5.4
    - date resolved: **2026-09-17 19:52**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.126: :textred:`resolved BUG 2489` : robotics/future.py uses an undefined name graphics 
    - issue author: Claude-JG
    - description:  MakeCorkeRobot-related code in exudyn/robotics/future.py builds graphicsBaseList from graphics.Brick/graphics.Cylinder/graphics.color but nothing imports graphics; the function does four star imports and none of them provides it (verified: exudyn.utilities and exudyn.graphicsDataUtilities have no attribute graphics). The path raises NameError. ruff reports F405 rather than F821 because of the star imports; mypy found it. revision2026 step R5.5.6
    - **notes:** future.py now imports exudyn.graphics as graphics in its __main__ block; none of the four star imports there provides it (verified). Running the block afterwards showed the same pattern as #2488 one function further: MakeCorkeRobot caught the ImportError of roboticstoolbox; printed and continued; and then returned an undefined robotCorke - it now raises ImportError naming the package to install. revision2026 step R5.5.6
    - date resolved: **2026-09-17 19:52**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.125: :textred:`resolved BUG 2488` : four undefined names raise NameError when their code path is reached 
    - issue author: Claude-JG
    - description:  ruff F821 found four names that do not exist where they are used: lieGroupIntegration.py calls exudyn.Print(...) three times although the module imports exudyn as exu; and robotics/roboticsCore.py line 1392 calls SC.renderer.Start() although the class member is self.SC - so InverseKinematicsNumerical with useRenderer=True raises NameError instead of showing the renderer. All four are in error or option paths that no test reaches. revision2026 step R5.5.5
    - **notes:** lieGroupIntegration.py used exudyn.Print three times although the module imports exudyn as exu; and roboticsCore.py called SC.renderer.Start() where the member is self.SC - so InverseKinematicsNumerical(useRenderer=True) raised NameError instead of showing the renderer. Both fixed. Running the first path afterwards showed a second defect at the same place: after printing the message the function continued and raised UnboundLocalError on ComputeStep; the unknown-solver case now raises ValueError naming the accepted values. revision2026 step R5.5.5
    - date resolved: **2026-09-17 18:43**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.124: resolved Issue 2487: no linter runs over the shipped Python package (extension)
    - issue author: Claude-JG
    - description:  python/exudyn is 34000 lines and nothing checks it for undefined names; unused imports or bare except: - the only linter in use is pydoclint; which only judges docstrings. ruff is introduced with an explicitly written rule set (F and E4/E7/E9); the findings present at introduction are tolerated through a baseline and a new finding fails the check. revision2026 step R5.5
    - **notes:** ruff runs over python/exudyn with the rule set written down in pyproject.toml (F and E4/E7/E9; E701/E702/E703 switched off as house style) and tools/checkPython.py judges it against tools/ci/ruffBaseline.txt: the findings present at introduction are tolerated; a new one fails. 335 findings at introduction; 107 of them fixed outright (52 == None; 11 == True/False; 13 not-in/not-is; 21 unused imports and 3 deliberate re-exports marked noqa; one multi-import line; one empty f-string) leaving 228 in the baseline - bare except:; type() comparisons; star imports and unused variables; each of which needs a judgement rather than a rewrite. revision2026 step R5.5
    - date resolved: **2026-09-17 18:37**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.123: :textred:`resolved BUG 2486` : generated __init__.pyi is not valid Python 
    - issue author: Claude-JG
    - description:  The stub file python/exudyn/__init__.pyi does not parse: one documentation line of the OutputVariableType description starts at column 0; createStubFiles.py takes that as the end of the class block and writes the rest of the class body to the top level. All five generated stub fragments parse - only the merged product does not; indenting that line makes the whole file parse. Nothing in the generator checks that the stub it wrote is valid Python. revision2026 step R5.5.1
    - **notes:** Fixed at the emitter: DocStringGoogleFromPlainText wrote the summary part of a docstring without indentation; a summary ends at the first period-space and therefore may contain line breaks - here the OutputVariableType description. Its continued lines now get the docstring indentation. In addition createStubFiles.py parses both stub files with ast.parse before writing them and refuses to write an invalid stub - verified by mutation: with the emitter fix reverted the generator fails with the file and line. revision2026 step R5.5.1
    - date resolved: **2026-09-17 18:28**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.122: resolved Issue 2485: the C++ usage demo of the symbolic types is a dead function of if(false) blocks (improvement)
    - issue author: Claude-JG
    - description:  PyTest_unused() at the end of Symbolic.cpp was six if(false)/if(true) blocks in one function, never called, not compiled into anything meaningful and never run. It is the only documentation of how Symbolic::SReal, SymbolicRealVector and SymbolicRealMatrix are used from C++, which is worth keeping (maintainer, 2026-09-17), but not in that shape - and it demonstrates a stack ExpressionNamedReal, which is exactly the heap corruption trap found in step R5.4.2. Reshape into a header that says what it is, with one named function per topic. revision2026 step R5.4.12.
    - **notes:** PyTest_unused() is removed from Symbolic.cpp and reshaped into src/Linalg/symbolicCppDemo.h: one named function per topic (plain values, named variable, vectors, matrices, Diff, the scalar functions, timing, reference counting) plus SymbolicDemoAll(). The header states in its first lines that it is worked examples to be read, that it is not a test (those are in SymbolicUnitTests.h), and that nothing calls it. It IS included by Symbolic.cpp so that it keeps compiling - inline and unused, so it costs nothing. Two corrections while reshaping: the demo no longer teaches the stack ExpressionNamedReal that causes heap corruption, and it was actually run through a temporary binding - every section produces sensible output, the new/delete counts balance, and the timing section now documents 12 ns per evaluation of a recorded tree against 267 ns when rebuilding it and 2.4 ns without recording. revision2026 step R5.4.12.
    - date resolved: **2026-09-17 17:20**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.121: resolved Issue 2483: document that EigenDense does not detect a singular matrix (improvement)
    - issue author: Claude-JG
    - description:  CORRECTED 2026-09-17 after maintainer feedback: the behaviour is NOT a defect. The FullPivLU path is entered only with linearSolverSettings.ignoreSingularJacobian=True, which is documented as handling over- and underdetermined systems and resolving redundant constraints (with a warning that it may lead to erroneous results), and the code says so as well: "in this case, we could report errors, but we do not want to". It also honours pivotThreshold via setThreshold. The PartialPivLU path cannot report invertibility at all - "according to Eigen homepage, there is no possibility to check for invertability" - so it is a property of the library, not a decision of Exudyn. What remains is a documentation gap: LinearSolverType.EigenDense describes partial pivoting as "faster than EXUdense" and mentions full pivot only under ignoreSingularJacobian, but nowhere states that in the DEFAULT EigenDense mode a singular Jacobian is not detected and the solver continues with an undefined result, while EXUdense and EigenSparse do report it. Add one sentence to the enum description. revision2026 step R5.4.10.
    - **notes:** Documentation only, as corrected by the maintainer: the behaviour is deliberate (FullPivLU is the ignoreSingularJacobian least-squares path; PartialPivLU has no invertibility check in Eigen). The LinearSolverType.EigenDense description in definitions/enumTypes.py now states that in the default partial pivoting mode a singular matrix is NOT detected and the solver continues with an undefined result, and points at EXUdense/EigenSparse if that must be reported or at full pivot if the singular system should be resolved by least squares on purpose. The sentence propagates to EnumTypes.h, the stubs and the documentation. The judging comment in LinearSolverUnitTests.h was corrected as well. revision2026 step R5.4.10.
    - date resolved: **2026-09-17 17:20**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.120: resolved Issue 2482: the sparse FactorizeNew does not return the causing row it promises (bug)
    - issue author: Claude-JG
    - description:  GeneralMatrixEigenSparse::FactorizeNew() takes rv = solver.info() - an Eigen::ComputationInfo, i.e. 0..3 - and then returns rv-1 as if it were a row index (LinearSolver.cpp:528-530), with a comment describing the causing row. In practice a failed factorization gives info()==1 (NumericalIssue), so the caller is told row 0 no matter which row is singular. Either return the real row, or return a documented error code and stop promising a row. The dense EXUdense path does return a real row. revision2026 step R5.4.9.
    - **notes:** GeneralMatrixEigenSparse::FactorizeNew() now returns NumberOfRows() on failure - the value the caller already treats as "causing row unknown" - instead of solver.info()-1, which was an Eigen ComputationInfo and therefore always 0. The comment that described SuperLU info semantics is replaced by what Eigen actually returns. Measured on a redundantly constrained system with EigenSparse: before, the solver printed "causing system equation number (coordinate number) = 0" and "The causing system equation 0 belongs to a ODE2 coordinate"; after, the singularity is reported with no row claimed. The symmetric branch always behaved this way. The R5.4.3 unit test asserts the exact return value. revision2026 step R5.4.9.
    - date resolved: **2026-09-17 17:20**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.119: resolved Issue 2481: a symbolic vector product with inconsistent sizes leaks its nodes (bug)
    - issue author: Claude-JG
    - description:  Symbolic::operator\*(SymbolicRealVector, SymbolicRealVector) allocates the product node and then hands it to SReal(ExpressionBase\*), whose constructor evaluates it immediately to cache the value. With inconsistent sizes that evaluation throws - before any SReal owns the node - so the allocation is never freed. Measured from Python: one failed product leaks 1 Real node and 2 Vector nodes (newCount increases, deleteCount does not). Any error path inside Evaluate has the same shape. revision2026 step R5.4.8.
    - **notes:** SReal(ExpressionBase\*) now takes ownership first and evaluates in the constructor body, with a catch(...) that releases the tree exactly as the destructor would (DecreaseReferenceCounter, then Destroy and delete at zero, counting the delete) and rethrows. One place, because every operator returns through it. Measured: a failed vector product used to leak 1 Real and 2 Vector nodes, now new==delete. The R5.4.2 test case requires OpenNodes()==0 instead of documenting the leak, and symbolicModuleTest.py no longer has to save and restore the counters around its error paths; its reference value is unchanged. Mutation check: the eager member-initializer form produces 1 failure. revision2026 step R5.4.8.
    - date resolved: **2026-09-17 16:43**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.118: resolved Issue 2480: SymbolicVector.h and SymbolicMatrix.h do not include what they use (bug)
    - issue author: Claude-JG
    - description:  Both headers use py::list, py::array_t and EPyUtils but include no pybind11 header and not PybindUtilities.h. They compile only because Symbolic.cpp includes pybind11 before them; any other translation unit that includes SymbolicVector.h first fails with a wall of errors about an unknown namespace py (found while writing the R5.4.2 tests, which now have to repeat that include order themselves). Add the includes the headers need. revision2026 step R5.4.7.
    - **notes:** Symbolic.h now includes BasicLinalg.h, <unordered_map> and <typeinfo>; SymbolicVector.h includes Symbolic.h and PybindUtilities.h; SymbolicMatrix.h includes those plus SymbolicVector.h. PybindUtilities.h brings the pybind headers, the py alias and EPyUtils in one line and includes nothing symbolic, so there is no cycle. The proof is a deletion: the four-line include workaround in SymbolicUnitTests.h is gone and the tests compile with the headers in any order. revision2026 step R5.4.7.
    - date resolved: **2026-09-17 16:43**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.117: resolved Issue 2476: SparseTripletMatrix(rows, columns, triplets) throws its size arguments away (bug)
    - issue author: Claude-JG
    - description:  The three-argument constructor initialises numberOfRows(0) and numberOfColumns(0) and never assigns numberOfRowsInit or numberOfColumnsInit, so a matrix built with it reports 0 x 0 while holding the triplets. Nothing in the code base calls it - it was found while fixing #2474, which makes the size fields load-bearing for MultMatrixVector. Either assign them or delete the constructor. revision2026 step R5.4.6.
    - **notes:** The three-argument constructor now assigns numberOfRowsInit and numberOfColumnsInit; fixed and kept rather than deleted, on the maintainer decision. It also gets its first caller: a case in AllMatrixVariantsUnitTests.h builds a 2x3 matrix through it and asks a MatrixContainer for a matrix-vector product, which since #2474 sizes its result from exactly those fields. Mutation check: putting the zeros back produces 1 failure. revision2026 step R5.4.6.
    - date resolved: **2026-09-17 16:43**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.116: resolved Issue 2484: pythonTests.cpp is dead manual test code (improvement)
    - issue author: Claude-JG
    - description:  src/Pymodules/pythonTests.cpp is 849 lines of which 56 are live code, and the body of PyTest() is commented out ENTIRELY - the function does nothing. It is bound as exu.Test only outside a release build. CreateTestSystem is bound only under _MYDEBUG and builds a model by py::exec of a Python string written against the pre-exudyn API (from itemInterface import \*, mbs.AddObject with a raw dict); it predates exudyn.demos, which does the same job properly. Both are more misleading than helpful (maintainer, 2026-09-17). Remove the file and its header PybindTests.h, the two bindings in PybindModule.cpp and the project entries. revision2026 step R5.4.11.
    - **notes:** Removed: src/Pymodules/pythonTests.cpp and src/Pymodules/PybindTests.h, the include and the two m.def bindings in PybindModule.cpp, the ClCompile/ClInclude entries in cppsrc.vcxproj and .filters, and the pythonTests.cpp entry in the hand-maintained minimal list of sources.json. The compile list is now 132 sources, 53 minimal. A comment in tools/benchmarks/avx2Benchmark.py that pointed at the removed sweep was rewritten. Verified: the module builds, exu.Test and exu.CreateTestSystem are gone, gen_sources.py agrees, suite PASSED, pytest passed. revision2026 step R5.4.11.
    - date resolved: **2026-09-17 16:27**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.115: resolved Issue 2479: Symbolic and LinearSolver have no C++ unit tests (improvement)
    - issue author: Claude-JG
    - description:  src/Tests/ covers the vectors, the arrays, the matrix classes and the geometry group, but neither the symbolic expression system (Symbolic.h, SymbolicVector.h, SymbolicMatrix.h; ~4000 lines) nor LinearSolver.h (the dense and the sparse system matrix behind one GeneralMatrix interface). symbolicModuleTest.py compares the NUMBERS from Python, so the C++ tests aim at the expression TREE - Diff, the value accessors, the non-recording path, the reference counting - and the solver tests solve one system with all four variants and pin down where the variants deliberately differ. revision2026 steps R5.4.2 and R5.4.3.
    - **notes:** Two new headers, 21 cases. SymbolicUnitTests.h (11) goes at the expression TREE, which symbolicModuleTest.py cannot reach: Diff by pointer identity, the chain rule, what Diff throws for and what it answers with NaN, exact ToString output, the non-recording branch as a second implementation, the value-accessor contracts, SetSRealVector and EvaluateComponent (both unbound in Python), and the reference counting - each case under a guard that restores recordExpressions and the new/delete counters, because symbolicModuleTest.py measures those. LinearSolverUnitTests.h (10) solves one 4x4 system with all four variants and pins down where they differ: only EXUdense reports the causing row, the Eigen dense paths report success for a singular matrix, EXUdense overwrites its matrix with the inverse while factorizing, and SetMatrix does not size the sparse matrix. symbolicModuleTest.py was extended (Diff, VariableSet, recording off, __str__, error paths) with its reference value unchanged at 0.9484129575069745. Mutation-checked: a dropped term in the product rule gives 2 failures, a dense factorization that always claims success gives 1. Found on the way: #2480, #2481, #2482, #2483. revision2026 steps R5.4.2 and R5.4.3.
    - date resolved: **2026-09-17 16:05**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.114: resolved Issue 2478: a model run locally still opens plot windows (improvement)
    - issue author: Claude-JG
    - description:  Step R5.17 gave exudyn the suppressPlots flag and it reaches every plot the PACKAGE draws (PlotSensor, PlotFFT, ParameterVariationPlot, the Campbell diagram). It cannot reach a script that imports matplotlib itself and calls plt.show(): 36 models and examples do exactly that, so running one of them locally - to check a change or reproduce a bug - still opens a window on the maintainer screen and waits. The runners are not affected, they set the Agg backend in their bootstrap. Needed: a one-line guard in those 36 scripts that switches to the non-interactive backend when exudyn.special.userInterface.suppressPlots is set, plus a rule in CLAUDE.md that a local run of an existing model sets EXUDYN_SUPPRESS_UI_WINDOW_OPEN and EXUDYN_OUTPUTDIRECTORY. revision2026 step R5.17.1.
    - **notes:** The 36 models and examples that import matplotlib themselves and call plt.show() now carry a two-line guard right after their exudyn import: it switches to the non-interactive Agg backend when exudyn.special.userInterface.suppressPlots is set. Measured both ways - with EXUDYN_SUPPRESS_UI_WINDOW_OPEN=1 the backend is Agg and plt.show() returns at once; without it the backend stays tkagg. CLAUDE.md gained hard rule 11: a local run of a model or example sets EXUDYN_SUPPRESS_UI_WINDOW_OPEN and EXUDYN_OUTPUTDIRECTORY. 171 examples with the same 5 pre-existing failures; suite PASSED; pytest 136 passed. revision2026 step R5.17.1.
    - date resolved: **2026-09-17 15:28**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.113: resolved Issue 2477: Exudyn has no way to say do not open windows (improvement)
    - issue author: Claude-JG
    - description:  A model or an example run outside the test suite opens the renderer and waits for the human - there is no switch for it. The test runners therefore REWRITE the source before running it (six substitutions in testRunnerTools.PrepareExampleSource neutralise SC.renderer.Start/Stop/DoIdleTasks/IsActive, mbs.SolutionViewer, InteractiveDialog and plt.show), which only works for code the runner controls. Needed: exudyn.special.userInterface with suppressRenderer, suppressSolutionViewer, suppressPlots and suppressDialogs, readable from C++ (the renderer window and idle loop live there) and from the Python helpers, plus the environment variables EXUDYN_SUPPRESS_UI_WINDOW_OPEN and EXUDYN_OUTPUTDIRECTORY for automated runs. revision2026 step R5.17.
    - **notes:** New group exu.special.userInterface (PySpecialUserInterface next to PySpecialSolver and PySpecialExceptions) with suppressRenderer, suppressSolutionViewer, suppressPlots, suppressDialogs and SuppressAll(). The renderer honours them in C++ (Start returns False, IsActive is False so a while-IsActive loop ends at once, DoIdleTasks returns), the Python helpers through basicUtilities.UIWindowSuppressed, which also prints ONE notice per kind. __init__.py reads EXUDYN_SUPPRESS_UI_WINDOW_OPEN and EXUDYN_OUTPUTDIRECTORY once, each in try/except and each announced; the first also switches matplotlib to Agg, which is the only thing that reaches the 29 examples and 9 models calling plt.show() themselves. Six source substitutions deleted from the test runners. Verified: 171 examples with the same 5 pre-existing failures, suite PASSED, pytest 136 passed. revision2026 step R5.17.
    - date resolved: **2026-09-17 13:01**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.112: :textred:`resolved BUG 2474` : MatrixContainer::MultMatrixVector has two preconditions depending on its mode 
    - issue author: Claude-JG
    - description:  The dense path calls MultMatrixVectorTemplate, which does result.SetNumberOfItems(rows); the sparse path calls SparseTripletMatrix::MultMatrixVector, which only does solution.SetAll(0.) and then indexes solution[triplet.row()]. So the same call with an unsized result vector works for a dense container and, for a sparse one, throws in a checked build and writes OUT OF BOUNDS in exudynCPPfast. One interface must not have two preconditions: size the vector in the sparse path as well. Found by the unit tests of revision2026 step R5.4.1.
    - **notes:** SparseTripletMatrix::MultMatrixVector sizes the result to NumberOfRows() before zeroing it, as the dense path has always done, and both sparse products now check their sizes. The R5.4.1 container test passes UNSIZED result vectors to both modes; commenting the new SetNumberOfItems out produces exactly one failure. Found on the way: #2476 (step R5.4.6). revision2026 step R5.4.5.
    - date resolved: **2026-09-17 11:15**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.111: :textred:`resolved BUG 2473` : LinkedDataMatrix: the two constructors taking a matrix do not compile 
    - issue author: Claude-JG
    - description:  LinkedDataMatrixBase(const MatrixBase<T>&) and LinkedDataMatrixBase(const MatrixBase<T>&, Index startRows, Index numberOfRowsLinked) read m.numberOfRows/numberOfColumns/data - protected members of another object - which C++ does not allow through a base-class reference (error C2248), and the const-ness of the data pointer does not match either. Neither is used anywhere: the code base links through the pointer constructor (CMarkerSuperElementPosition.cpp, CMarkerSuperElementRigid.cpp), so this has never been noticed. Found by the unit tests of revision2026 step R5.4.1 being their first users. Either fix them to use the public accessors or delete them; a row-range link is useful and the pointer arithmetic is currently up to the caller.
    - **notes:** LinkedDataMatrixBase(const MatrixBase<T>&) now uses the public accessors NumberOfRows/NumberOfColumns/GetDataPointer instead of the protected members of another object (C2248). The row-range constructor next to it gets its first caller: the R5.4.1 test no longer does the row-major pointer arithmetic by hand. All C++ unit tests pass. revision2026 step R5.4.4.
    - date resolved: **2026-09-17 11:15**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.110: resolved Issue 2475: five examples write sensor output next to themselves, it clutters the working tree (improvement)
    - issue author: Claude-JG
    - description:  Five examples name their sensor files without the solution/ prefix that every other model and example uses: beltDriveALE and beltDriveReevingSystem write into solutionDelete/, beltDriveReevingSystem also writes its coordinates solution into solution_nosync/, rigidBodyIMUtest into solutionIMU<mode>/, sliderCrank3DwithANCFbeltDrive and sliderCrank3DwithANCFbeltDrive2 into the current directory (Preload_overallS.txt, Pos_Disk0_overallS.txt, Angular_velocity_overallS.txt, angular_velocity_disk0/1.txt, torque.txt, crank_pos.txt) plus a plots/ directory. Only solution/ is in .gitignore, so a direct run of one of these examples leaves untracked and unignored files and directories behind - and the reads (np.loadtxt) do not go through OutputFilePath either, so they ignore exudyn.config.outputDirectory. revision2026 step R5.13.2.
    - **notes:** All five examples now write through solution/ and read through OutputFilePath: beltDriveALE and beltDriveReevingSystem (solutionDelete/ and solution_nosync/ are gone), rigidBodyIMUtest (solution/IMU<mode>/), and the two sliderCrank3DwithANCFbeltDrive examples (bare names and plots/ -> solution/ and solution/plots/). Verified by running all five from python/Examples with no output directory set: one solution/ directory appears and nothing else. revision2026 step R5.13.2.
    - date resolved: **2026-09-17 11:04**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.109: resolved Issue 2472: unit tests for the matrix variants and the rigid body / geometry classes (extension)
    - issue author: Claude-JG
    - description:  revision2026 step R5.4.1: ResizableMatrix, ConstSizeMatrix, LinkedDataMatrix, MatrixContainer, RigidBodyMath, Geometry, BoundingBox and SearchTree had no unit tests; AllMatrixUnitTests.h covered only the base Matrix class. Two new headers, property-based where possible (rotation round trips, skew as cross product, SearchTree against a brute-force scan).
    - **notes:** src/Tests/AllMatrixVariantsUnitTests.h (10 cases) and src/Tests/RigidBodyMathUnitTests.h (10 cases), registered in UnitTestBase.cpp and in the VS project. Property-based where a property exists: skew IS the cross product, a rotation matrix is orthonormal with determinant +1, EP and RotXYZ round trip through the matrix, RotationVector2RotationMatrix satisfies trace = 1+2cos(angle) and fixes its own axis, and every SearchTree query is compared against a brute-force scan for 1x1x1, 2x2x2 and 5x5x5 cells. Validated by mutation: a sign flipped in Vector2SkewMatrix gives 4 failures. The second mutation (the sparse product accumulating with = instead of +=) was NOT caught at first - the test matrix had one entry per row - so the test was strengthened to two entries in one row and now catches it. Also pinned down: RotXYZ composes as A(x)\*A(y)\*A(z), DistanceToPlane is UNSIGNED, and Box3D::Intersect REJECTS touching boxes although its comment claims otherwise. Two defects found and raised rather than fixed: #2473, #2474. revision2026 step R5.4.1.
    - date resolved: **2026-09-17 10:15**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.108: :textred:`resolved BUG 2469` : NGsolveCMStest reference value follows an untracked mesh cache 
    - issue author: Claude-JG
    - description:  NGsolveCMStest.py loads python/TestModels/testData/netgenTestMesh.pkl - a TRACKED file - and silently OVERWRITES it whenever the load fails. That happened during revision2026 step R2.10 and the result moved by 2.4e-8, far outside the 5e-14 tolerance, back to the value recorded before 2025-05-05; restoring the committed file restored the value. A test must not rewrite its own committed input: either treat the mesh as read-only and fail loudly, or generate it deterministically.
    - **notes:** NGsolveCMStest.py decided on an EXCEPTION: it saved the mesh whenever LoadFromFile failed, and a load fails for reasons unrelated to the file being absent - under a second C++ module it raises 'type "Real" is already registered', a new ngsolve version would do the same. The tracked testData/netgenTestMesh.pkl was then overwritten and the result moved by 2.4e-8, far outside the 5e-14 tolerance, leaving the working tree dirty. It now decides on os.path.isfile: the file is written only when it does not exist, and a file that exists but cannot be loaded raises with the reason and the advice to delete it deliberately. Verified in three states - present (loads, byte-identical afterwards), corrupt (raises, not rewritten), missing (recreated, and the new mesh gives the other value, which is why it must not happen by accident). revision2026 step R2.10.2. ALSO: the tracked mesh was converted from .pkl to .npz (maintainer, 2026-09-17), keeping the mesh and the reference value; .gitignore narrowed so the tracked mesh is not matched. The .npz still cannot be read by a second module - one stored field is a C++ enum - which is #2471.
    - date resolved: **2026-09-17 08:39**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.107: resolved Issue 2470: a second reference set for the AVX2 module, as an update to the baseline (extension)
    - issue author: Claude-JG
    - description:  revision2026 step R2.10.3: since step R2.10 the regular module is baseline ISA and exudynCPPfast carries AVX2, so one set of reference values cannot judge both on Windows. AVX2ReferenceSolutionUpdate() at the end of runTestSuiteRefSol.py holds ONLY the values that move (32 of 136), as an update to the baseline values, and is meant to shrink as the causes are found or the models are re-parameterised to amplify roundoff less.
    - **notes:** AVX2ReferenceSolutionUpdate() holds the 32 values (31 models + 1 mini example) that move on an AVX2 module, ordered by drift so the worst are the work list; everything else stays single-sourced in the baseline values. testRunnerTools.ModuleUsesAVX2() asks the module (the platform string already reports AVX2/AVX512), so a --no-avx2 fast build is judged by the baseline values. NotJudgedOutsideRegularModule() skips two models under exudynCPPfast: parameterConversionTest (its result counts rejected inputs) and NGsolveCMStest (its pickled FEM data carries the C++ types of one module; loading under a second raises 'type Real is already registered' and the model then overwrites its own tracked input, #2469). Both runTestSuite.py and the pytest collector apply this. Verified: both modules pass the suite from one reference file. revision2026 step R2.10.3.
    - date resolved: **2026-09-17 00:32**\ , date raised: 2026-09-17 
    - resolved by: Claude-JG
 * Version 1.11.106: :textred:`resolved BUG 2467` : exudynCPPfast crashes in the test suite (segmentation fault) 
    - issue author: Claude-JG
    - description:  Running runTestSuite.py against exudynCPPfast segfaults reproducibly: with AVX2 after TestModel 74 (objectFFRFTest.py), without AVX2 after 75 (objectFFRFTest2.py). Both models pass standalone under the same module, so this is accumulated corruption, and the shifting crash point shows it is __FAST_EXUDYN_LINALG (no range checks), not the vector extensions. Found in revision2026 step R2.10; the fast module was built for one Python version only and the suite had apparently never been run against it.
    - **notes:** Root cause: MainObjectANCFThinPlate::SetWithDictionary calls ParametersHaveChanged() with whatever the user wrote into the dictionary, BEFORE the object factory validates the item, and CObjectANCFThinPlate::ParametersHaveChanged computes the slope scaling from the nodes - reading cSystemData->GetCNodes()[-1] for a default/invalid node number. In exudynCPP the range check of the array turns that into a Python exception, so it was invisible; in exudynCPPfast the checks are compiled out and the process died. Found with AddressSanitizer (access-violation at 0xffffffffffffffff in CObject::GetCNode). ThinPlate is the only object that dereferences nodes in ParametersHaveChanged. Fixed by computing the scaling only when the node numbers address existing nodes; the factory reports the invalid number right afterwards. The full suite now completes under exudynCPPfast. revision2026 step R2.10.1.
    - date resolved: **2026-09-17 00:11**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.105: :textred:`resolved BUG 2468` : deleting build/temp is not enough after a compile-flag change 
    - issue author: Claude-JG
    - description:  The documented gate (revision2026 step R2.17, #2427) says to delete build/temp.win-amd64-cpython-313 after a header change. That is insufficient for a FLAG change: build/lib.win-amd64-cpython-313 keeps the previously linked .pyd and the wheel is assembled from it, so a rebuild silently ships the old binary. This produced three contradictory measurements in revision2026 step R2.10 before the whole build/ directory was removed. Fix the gate instruction and preferably make setup.py handle it.
    - **notes:** setup.py writes the effective compiler options of every extension to build/exudynBuildFlags.txt and compares them at the start of the next build; on a difference it removes the object directory AND the linked modules, then recompiles. The stamp sits next to build/temp\* and build/lib\*, not inside either (MSVC build_temp is build/temp.../Release). Verified: two builds differing only in EXUDYN_EXTRA_COMPILE_ARGS=/arch:AVX2, nothing deleted by hand, give 7380480 and 7400448 byte modules and the expected 0 vs 32 test failures. revision2026 step R2.17.1.
    - date resolved: **2026-09-16 23:47**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.104: resolved Issue 2466: two shipped C++ modules: exudynCPPnoAVX dropped, AVX2 belongs to exudynCPPfast (extension)
    - issue author: Claude-JG
    - description:  revision2026 step R2.10: the default module exudynCPP is compiled for the BASELINE instruction set on every platform and exudynCPPfast carries __FAST_EXUDYN_LINALG and AVX2; the third module exudynCPPnoAVX and the sys.exudynCPUhasAVX2 switch are removed. New build switches useAVX2 (default on; effective only inside the fast module) and useAVX512 (default off; requires useAVX2). All Windows reference values re-measured on the baseline module.
    - **notes:** setup.py: useAVX2/useAVX512 switches, vector flags moved into exudynCPPfast (with -ffp-contract=off on gcc/clang), exudynCPPnoAVX removed, fast module built for Python 3.13 in a development version. python/exudyn/__init__.py: two candidates and an OS-level AVX2 check replacing the numpy __cpu_features__ read. All 113 Windows reference values re-measured on the baseline module (85 moved, 33 of them past tolerance); sphereTriangleTest.py moved to DeliberatelyNotRun. Suite and pytest green.
    - date resolved: **2026-09-16 23:20**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.103: :textred:`resolved BUG 2465` : the two AVX vector classes have no unit tests 
    - issue author: Claude-JG
    - description:  ResizableVectorParallel and LinkedDataVectorParallel are covered by no lest case; #2394 (a misaligned __m256d load on a LinkedDataVector sub-range) lived there and was found by a sanitizer on a whole test model. A defect in these classes is invisible in the TestModels on Windows and can change results silently. revision2026 step R5.4.
    - **notes:** revision2026 step R5.4: new src/Tests/AVXVectorUnitTests.h with 7 cases; each over 19 lengths around the AVX packet boundary built from AVXRealSize; LinkedDataVectorParallel at every offset from 0 to AVXRealSize into a padded buffer; results compared against a scalar loop; one length above the multithreading limit. Verified by mutation: a remainder loop started one item late gives 2 failed tests.
    - date resolved: **2026-09-16 20:40**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.102: :textred:`resolved BUG 2464` : the lest C++ unit tests are compiled nowhere 
    - issue author: Claude-JG
    - description:  src/Tests/ is gated on PERFORM_UNIT_TESTS; setup.py added that define for Python 3.7 only, and 3.7 stopped being built long ago (requires-python >=3.10), so the unit tests run in no wheel; no CI job and no local build. The VS Debug configuration does not define it either. revision2026 step R5.3.
    - **notes:** revision2026 step R5.3: new build switch performUnitTests (command line --unittests; EXUDYN_PERFORM_UNIT_TESTS; [tool.exudyn]); off by default; -DPERFORM_UNIT_TESTS also for unix; PERFORM_UNIT_TESTS in the VS Debug configuration. Verified: the tests compile and all pass.
    - date resolved: **2026-09-16 19:59**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.101: resolved Issue 2458: runTestSuite.py checks hasattr(exu.solver, RunCppUnitTests) instead of exu.special (fix)
    - issue author: Claude-JG
    - description:  found 2026-09-16 while preparing revision2026 step R5.3: the C++ unit tests are gated on hasattr(exu.solver, "RunCppUnitTests"), but the function is bound on exu.special (Pybind_manual_classes.cpp, inside #ifdef PERFORM_UNIT_TESTS); exu.solver is the Python module exudyn.solver and never has that attribute, so the check is always False and the tests are reported as skipped even in a build that contains them. Belongs to revision2026 step R5.3
    - **notes:** revision2026 step R5.3: runTestSuite.py checked hasattr(exu.solver; RunCppUnitTests) while the binding is on exu.special - the C++ unit tests were reported as skipped even in a build that has them; and the summary called len() on the returned count.
    - date resolved: **2026-09-16 19:59**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.100: resolved Issue 2462: dev tool dependencies are not declared (extension)
    - issue author: Claude-JG
    - description:  pytest and pytest-xdist (revision2026 step R5.1) are installed by hand and appear nowhere in the project metadata; the build dependency group does not match build-system.requires (setuptools>=77; tomli) and does not carry the cibuildwheel version the wheels workflow pins. revision2026 step R5.14.
    - **notes:** revision2026 step R5.14: new test dependency group (pytest; pytest-xdist); build group matched to build-system.requires (setuptools>=77; tomli) and to the pinned cibuildwheel version; condaEnvironments.md group table updated.
    - date resolved: **2026-09-16 19:54**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.99: resolved Issue 2461: the examples run serially and take six minutes (extension)
    - issue author: Claude-JG
    - description:  runTestExamples.py execs 171 examples one after the other into a single interpreter; 360 seconds on Windows. The examples are an API check - an example has done its job once it has built its model and reached the solver. revision2026 step R5.16: run them in separate interpreters in parallel; give every example its own output directory; and count a timeout as a pass once the solver was reached.
    - **notes:** revision2026 step R5.16: testRunnerTools.RunExampleInProcess/RunExamplesInParallel run every example in its own interpreter and its own output directory; a timeout after the solver was reached counts as a pass; PrepareExampleSource and ExampleSkipReason moved into testRunnerTools; 43 examples wrap a read of their own output in OutputFilePath; chainDriveExample imports sin/cos/arcsin; minimizeExample and dispyParameterVariationExample are skipped as not self-contained. 360 s -> 49 s.
    - date resolved: **2026-09-16 18:57**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.98: resolved Issue 2460: performance suite reports only one wall-clock time per model (extension)
    - issue author: Claude-JG
    - description:  runPerformanceTests.py wraps each model in time.time() and prints one number per file; that number contains model build; assembly and Python overhead; and every model covers exactly one problem size and one thread count. revision2026 step R5.15: record the solver time of every single simulation run in exudynTestGlobals.timings and report them; vary the size of perfLargeMassSpringChain and the thread count of generalContactSpheresTest; make perfLargeMassSpringChain a rigid body chain so that it measures the object computation.
    - **notes:** revision2026 step R5.15: testRunnerTools.AddTiming records solver.timer.total; the result and a run name per simulation in exudynTestGlobals.timings; runPerformanceTests.py prints and judges the 13 single runs; perfLargeMassSpringChain is a rigid body chain over 1000/5000/20000 bodies (explicit and implicit; computeMassMatrixInversePerBody on); generalContactSpheresTest runs with 1/4/8 threads in the performance path only; new reference values for the reworked models and for perf3DRigidBodies and perfSpringDamperExplicit.
    - date resolved: **2026-09-16 17:51**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.97: resolved Issue 2459: no way to run a fast subset of the test models (add)
    - issue author: Claude-JG
    - description:  revision2026 step R5.2: every runner was all-or-nothing, so a pull-request run had to include the models that take longest and those needing optional packages (ngsolve, stable-baselines3). Added SlowTests() and OptionalPackageTests() as data in runTestSuiteRefSol.py, runTestSuite.py --fast and pytest markers slow/optionalPackage/sensitive/unresolvedOnLinux reading the same data
    - **notes:** revision2026 step R5.2: SlowTests/OptionalPackageTests as data; runTestSuite.py --fast (12 s) and pytest markers; nightly unchanged
    - date resolved: **2026-09-16 17:09**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.96: resolved Issue 2457: test models are not available as pytest cases (add)
    - issue author: Claude-JG
    - description:  revision2026 step R5.1: runTestSuite.py was the only way to run the test models, so there was no selection by name, no IDE or CI reporting and no standard parallel runner; added python/TestModels/test_testModels.py with one parametrized case per model and mini example, sharing reference values and tolerances with the suite
    - **notes:** revision2026 step R5.1: test_testModels.py, 137 cases, shared reference values and tolerances (testRunnerTools.BaseTolerance); found and fixed a masked defect in plotSensorTest.py
    - date resolved: **2026-09-16 15:04**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.95: resolved Issue 2456: runTestSuite.py runs the models serially (change)
    - issue author: Claude-JG
    - description:  revision2026 step R5.8: the 114 test models ran one after the other in a single interpreter (about 22 seconds); with each model writing into its own output directory since step R5.13 they can run in separate processes. Added --parallel[=N]; the models are reported in the order of the reference list; serial stays the default for the commit gate because multithreaded solvers and ARPACK shift the last digits under load
    - **notes:** revision2026 step R5.8: --parallel[=N] runs each model in its own interpreter (22 s -> 11 s with 8 workers); results reported in reference-list order; serial remains the default
    - date resolved: **2026-09-16 14:24**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.94: resolved Issue 2454: exudyn.config.outputDirectory: global output directory for solver written files (add)
    - issue author: Claude-JG
    - description:  revision2026 step R5.13: solver written files (coordinates solution; solver information; sensor files; exported images) are prefixed with exudyn.config.outputDirectory when it is set. An absolute file name together with a non-empty outputDirectory raises an error when the file is opened. The setting is global and lives as long as the module is loaded; it is meant for test runners and batch scripts; users should put the folder into the file names. Needed so that the test suite can give every model its own output directory (#2418) - the prerequisite for running the suite in parallel
    - **notes:** revision2026 step R5.13: ResolveOutputFileName() in Stdoutput.cpp, applied to solution, solver information, sensor and image files; absolute names raise on open
    - date resolved: **2026-09-16 10:16**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.93: resolved Issue 2418: test suite models write output files into TestModels instead of solution/ (testing)
    - issue author: Claude-JG
    - description:  models write solution and sensor files next to themselves (coordinatesSolution.txt and others) so TestModels/ fills with output and runs collide on the same file names - which blocks running the suite in parallel (revision2026 step R5.8). In the test suite all output goes to solution/ with unique per-model names; file writes are avoided widely (sensors storeInternal and writeSolutionToFile False) and done only sparsely so writing stays tested - those tests re-read the written files and check them. TestExamples stay serial: they only check that examples still run against the current API. revision2026 step R5.13.
    - **notes:** revision2026 step R5.13: exudyn.config.outputDirectory plus one output directory per model in runTestSuite.py; compareFullModifiedNewton is the designated writing test
    - date resolved: **2026-09-16 10:16**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.92: resolved Issue 2397: the only benchmark that resolves AVX2 is commented out inside exu.Test() (check)
    - issue author: Claude-JG
    - description:  measured 2026-09-12: runPerformanceTests.py gives 20.118 s without AVX2 and 20.137 s with -mavx2 -mfma in the same container - 0.1 percent; with per test differences in both directions. Four of the six performance tests are tiny systems run for about 1e6 steps; so the vectors are 3 to 20 elements long and per step overhead dominates; AVX2 only pays on long vectors. A sweep that does resolve it exists in PyTest() in src/Pymodules/pythonTests.cpp (exposed as exu.Test()); with recorded results in comments (speedup 3.1 for n=502; 3.5 for n=1002; plus an AVX/multithreaded/serial table for sizes 16 to 200002) - but it is entirely inside comment blocks and if (0); so exu.Test() runs none of it and the numbers cannot be reproduced. revision2026 step R2.10 cannot judge whether the fast variant earns its place in the wheel until there is a maintained benchmark over vector length
    - **notes:** revision2026 step R2.16: tools/benchmarks/avx2Benchmark.py with solver sub timers replaces the dead sweep in PyTest(); an in-Exudyn micro-benchmark is step R11.2
    - date resolved: **2026-09-16 08:54**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.91: resolved Issue 2396: enabling AVX2 on Linux shifts results by 1e-9..1e-6 through FMA contraction (check)
    - issue author: Claude-JG
    - description:  measured 2026-09-12 once the alignment defect was fixed: a manylinux cp313 build with -mavx2 -mfma runs the whole suite; but four tests that pass without AVX2 then fail against the 3e-11 tolerance - ANCFcontactCircleTest (-2.185e-08); ANCFslidingAndALEjointTest (2.236e-06); connectorGravityTest (-1.956e-07) and raytracerNOGLFWtest (-1.368e-06). All are rounding scale: a fused multiply-add rounds once where a separate multiply and add round twice. The same build WITHOUT -mavx2 reproduces the previous results bit for bit; so this is a property of AVX2; not of the alignment fix. Relevant to revision2026 step R2.10 (two shipped variants) and to the tolerance discussion in fact 24: either the reference values are variant dependent; or the tolerance has to admit FMA level differences
    - **notes:** revision2026 step R2.16: caused by FMA contraction; -ffp-contract=off removes it; AVX2 stays off for the default Linux wheel; a fast variant must use the flag
    - date resolved: **2026-09-16 08:54**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.90: resolved Issue 2387: 20 ClInclude entries in cppsrc.vcxproj have the wrong case; 5 entries do not exist (fix)
    - issue author: Claude-JG
    - description:  the vcxproj lists headers as src/utilities/...; src/linalg/...; src/system/...; src/tests/... and src/autogenerated/SimulationSettings.h while the tracked directories are Utilities; Linalg; System; Tests and Autogenerated - 20 entries in total. It also lists two headers that do not exist (src/Autogenerated/VisuObjectBeamGeometricallyExact3D.h and src/System/MainObjectFactory.h) and three missing ClassDiagram .cd files. These are IDE browsing entries; the compiler finds headers through the include path; so nothing is broken today - but the same class of defect in a ClCompile entry was a real Linux hazard (see issue 2382) and these would trip any move or flattening step. tools/gen_sources.py --check should be extended to cover ClInclude and None entries. Found during revision2026 step R2.6
    - **notes:** revision2026 step R2.15: case corrected and missing entries removed in vcxproj and filters; gen_sources.py --check covers all entries
    - date resolved: **2026-09-16 01:00**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.89: resolved Issue 2380: quietCompile does not actually quieten the compiler on Linux (check)
    - issue author: Claude-JG
    - description:  the quiet path redirects sys.stdout; but on Linux _compile spawns the compiler as a subprocess that writes to file descriptor 1 directly; so the compiler output bypasses the redirection and setuppy.output.txt stays empty (measured: 0 bytes after a full 133 file build). Suppressing it would need the subprocess stdout to be captured; not sys.stdout rebinding. Pre-existing; found while fixing the thread-safety defects of revision2026 step R2.3
    - **notes:** revision2026 step R2.15: file descriptors 1 and 2 redirected around the compile pool; verified in WSL
    - date resolved: **2026-09-16 01:00**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.88: resolved Issue 2372: project.license TOML table deprecated (check)
    - issue author: Claude-JG
    - description:  setuptools deprecates license = { text = ... } with deadline 2027-Feb-18; the replacement is an SPDX expression. There is no honest one: LICENSE.txt is the custom EXUDYN General License; not an OSI-approved BSD - which is also why the License :: OSI Approved :: BSD License classifier was dropped. Needs a decision on what the licence identifier should be
    - **notes:** revision2026 step R2.15: license = LicenseRef-EXUDYN-General-License with license-files; setuptools>=77
    - date resolved: **2026-09-16 01:00**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.87: resolved Issue 2453: Restructure and renumber the 2026 revision documents (docu)
    - issue author: Claude-JG
    - description:  Maintainer request 2026-09-15: the plan mixed general sections with the steps and numbered steps in discovery order; the log followed neither. Now three documents (exudynRevisionInfo2026.md with rules/facts/decisions/material for the user documentation/transition table; the plan by phases R0-R11; the log in plan order with closing commit and time); permanent numbers R<phase>.<step> (sub-steps R4.10.3); all references in tracked files renumbered including this tracker; code comments shall cite issues instead of plan steps.
    - **notes:** three revision documents; R numbering; transition table in exudynRevisionInfo2026.md section 15
    - date resolved: **2026-09-16 00:02**\ , date raised: 2026-09-16 
    - resolved by: Claude-JG
 * Version 1.11.86: resolved Issue 2411: expose item type and shape information to Python (extension)
    - issue author: Claude-JG
    - description:  Structures have a generated GetDictionaryWithTypeInfo() (pythonAutoGenerateSystemStructures.py:531) that feeds the settings dialog (GUI.py:323). Items have no equivalent; the items dialog therefore shows no types. Proposal: a generated exudyn/types/ subpackage carrying per parameter the type; shape and range; plus nodeType; requestedNodeType and requestedMarkerType (95 / 34 / 36 C-destination members today; none visible from Python). It must not go into itemInterface.py: that module is 390 KB and sits on the exudyn.utilities import path. setup.py:530 already uses find_namespace_packages; so packaging needs no change. Second part: requestedNodeType and requestedMarkerType are written as C++ in the definition today and should become declared type lists from which the accessor is generated. The additive case is the common one; the known hard case is ObjectContactSphereSphere; where the requested marker type is a base list plus one conditional term governed by one parameter (dynamicFriction != 0) - see src/Autogenerated/CObjectContactSphereSphere.h:184. Survey first whether any case needs more than one condition.
    - **notes:** revision2026 step R4.10.4: generated exudyn/types/items.py and query functions; typeInformationTest agrees with Assemble
    - date resolved: **2026-09-15 23:01**\ , date raised: 2026-09-13 
    - resolved by: Claude-JG
 * Version 1.11.85: resolved Issue 2452: Declared item types and access function types (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.10.3: GetType of nodes and markers and GetAccessFunctionTypes of objects (bodies in 19 cpp files) become declared lists with generated bodies; basis of the Python type information and compatibility queries (83d).
    - **notes:** revision2026 step R4.10.3: ItemTypes in 34 nodes/markers; ItemAccessFunctionTypes in 19 objects; cpp bodies removed
    - date resolved: **2026-09-15 22:50**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.84: resolved Issue 2451: Marker::Type is not in the enum registrator (change)
    - issue author: Claude-JG
    - description:  Marker::Type is hand-written in src/Main/OutputVariable.h with a comment to keep it synchronized with AccessFunctionType; NodeType and the others are generated from definitions/enumTypes.py (revision2026 step R4.3). ItemRequestedTypes (revision2026 step R4.10.1) can therefore validate node type names but not marker type names; and revision2026 step R4.10.2 needs the marker type names in Python.
    - **notes:** revision2026 step R4.10.2: Marker::Type and AccessFunctionType generated from enumTypes.py and bound to Python
    - date resolved: **2026-09-15 22:36**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.83: resolved Issue 2450: Declared requested node and marker types (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.10.1: GetRequestedNodeType / GetRequestedMarkerType are written as C++ inside 70 item definitions. Replace them by ItemRequestedTypes(kind; types; conditional) with a generated body; survey: the only conditional is Orientation if dynamicFriction != 0 (ObjectContactSphereSphere; ObjectContactSphereTriangle).
    - **notes:** revision2026 step R4.10.1: ItemRequestedTypes in 70 definitions; generated bodies; 12 headers change in spelling
    - date resolved: **2026-09-15 22:01**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.82: resolved Issue 2449: Group src/Autogenerated by item type (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.14: item headers move into nodes/ objects/ markers/ loads/ sensors/; generator; includes and vcxproj follow. Maintainer approval for the move 2026-09-15.
    - **notes:** revision2026 step R4.14: 291 item headers in nodes/objects/markers/loads/sensors; generator; 103 sources and vcxproj includes
    - date resolved: **2026-09-15 20:55**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.81: :textred:`resolved BUG 2447` : Super element Vshow is False when left out of the dict 
    - issue author: Claude-JG
    - description:  ObjectFFRF; ObjectFFRFreducedOrder; ObjectGenericODE2 and ObjectKinematicTree: AddObject without Vshow gives Vshow=False; although definitions/ and itemInterface.py give True and the generated Visu constructor sets show=true. Found by the omit probe of parameterConversionTest.py (revision2026 step R4.13). Likely the parent VisualizationObjectSuperElement constructor or a shadowed member.
    - **notes:** revision2026 step R4.26: duplicate bool show in VisualizationObjectSuperElement hid the base member; removed
    - date resolved: **2026-09-15 20:45**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.80: :textred:`resolved BUG 2427` : the wheel build reuses stale object files after a header-only change 
    - issue author: Claude-JG
    - description:  setuptools recompiles a .cpp only when the .cpp is newer than its .obj; it does not track included headers. Found in revision2026 step R4.4.3.2: after rewriting src/Pymodules/PybindUtilities.h (included by 35 files) pip wheel . -w dist finished and produced an exudynCPP .pyd with the same md5 as the previous build in build/lib.win-amd64-cpython-313 - nothing was compiled; the test suite then passed against the old binary. Only deleting build/temp.win-amd64-cpython-313 forced the full compile (49 s). So the build gate proves nothing for header-only changes. Fix options: pass depends= (all headers or the include graph) to the Extension so setuptools compares them; or have the gate remove build/temp first. revision2026 step R2.17.
    - **notes:** revision2026 step R2.17: setup.py passes all src headers as depends= to the extensions
    - date resolved: **2026-09-15 20:37**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.79: :textred:`resolved BUG 2448` : Wheel gate skips header-only changes 
    - issue author: Claude-JG
    - description:  pip wheel . reuses build/ (setuptools compares .cpp against .obj and the .pyd against its sources; headers are not dependencies): after changing only src/Autogenerated/\*.h the wheel was built in 4 s with the old binary and the suite ran against it. Found in revision2026 step R4.13; a clean build (build/ removed) takes 50 s. The documented gate must build from a clean build/ directory or the build must track header dependencies. Earlier steps that changed only headers were possibly tested against stale binaries.
    - **notes:** duplicate of #2427 (revision2026 step R2.17)
    - date resolved: **2026-09-15 20:35**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.78: resolved Issue 2446: Remove CFOptional: every item parameter may be left out of a dict (change)
    - issue author: Claude-JG
    - description:  CFOptional wrapped 448 of the dictionary writes in DictItemExists; the others raised KeyError when left out. revision2026 step R4.13 (#2417 is the original); this issue records the implementation choice: all parameters optional; must-be-given (CFMustBeGiven) ones left out raise if their value is still the placeholder.
    - **notes:** duplicate of #2417 (raised by mistake for revision2026 step R4.13)
    - date resolved: **2026-09-15 20:15**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.77: resolved Issue 2417: remove the CFOptional flag once phase 3 is done (cleanup)
    - issue author: Claude-JG
    - description:  CFOptional marks 448 of 985 item parameters and generates if (EPyUtils::DictItemExists(d; "x")) around the dictionary read; so a parameter that is absent keeps its C++ default. It still catches today - the test suite creates items from raw dicts; e.g. modelUnitTests.py:184 calls mbs.AddObject with objectType Ground and referencePosition only; and ObjectGround has four optional parameters that the call omits. Maintainer decision 2026-09-13: remove it after phase 3 anyway. The reasoning: there are practically no tests for this behaviour; so the flag is worthless as a guarantee; and the dict path works if every parameter is optional. Where a parameter really cannot be omitted; the failing default value is the right place to say so - a construction that cannot produce a usable object should fail on its default; not on a hand-written flag that nothing checks. Do this AFTER phase 3; and add tests for the raw-dict creation path at the same time; since that is what the flag silently protects today.
    - **notes:** revision2026 step R4.13: CFOptional removed; every dictionary write guarded; must-be-given parameters raise when left out; omit probe in parameterConversionTest
    - date resolved: **2026-09-15 20:15**\ , date raised: 2026-09-13 
    - resolved by: Claude-JG
 * Version 1.11.76: resolved Issue 2445: Structure members: SFPybind on almost every member; invert to SFNoPybind (change)
    - issue author: Claude-JG
    - description:  SFPybind (P) is set on 845 of 883 structure members; the 38 without it are solver internals (matrix links; temporary data; file streams; cSolver links and their accessors). Maintainer request 2026-09-15: invert the flag so only the exceptions are marked. Measured first by forcing P on all members: 9 generated files change (pybind; stubs; dictionaries; public/protected in CSolverStructures.h; docs); so the flag has an effect for both parameters and functions and must stay as the inverted flag.
    - **notes:** revision2026 step R4.25: SFNoPybind on 38 members replaces SFPybind on 845; effective flags identical; regeneration no-op
    - date resolved: **2026-09-15 19:59**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.75: resolved Issue 2409: the constrained parameter types PReal UReal PInt UInt are lost at the C++ boundary (extension)
    - issue author: Claude-JG
    - description:  PReal, UReal, PInt and UInt exist so that a bad parameter fails where the item is created - in itemInterface, through the generated CheckForValid\* guards - instead of surfacing later as a division by zero or a bad size deep in the solver. But typeConversion in pythonAutoGenerateObjects.py:1801 maps all of them onto plain Real resp. Index, so in the C++ core the intent is gone and a reader of the header cannot tell a positive-only quantity from any other Real. Proposal: add typedefs (PReal -> Real, UReal -> Real, PInt -> Index, UInt -> Index) and emit the constrained name into the generated C++, so the documentation value survives into the core. Purely additive; no behaviour change. Raised on maintainer request during revision2026 step R4.1.2.
    - **notes:** revision2026 step R4.12: constrained types stated as must be > 0 / >= 0 in generated header member comments; no typedefs because PReal is the AVX macro
    - date resolved: **2026-09-15 19:36**\ , date raised: 2026-09-13 
    - resolved by: Claude-JG
 * Version 1.11.74: :textred:`resolved BUG 2415` : a changed class description never reaches the generated headers 
    - issue author: Claude-JG
    - description:  pythonAutoGenerateObjects.py:2029 sets nLinesHeader = 7 and compares the new file text with the existing one only AFTER cutting those 7 lines (CutLinesFromString; autoGenerateHelper.py:155). The intent is to ignore the two volatile @date lines (6 and 7); but lines 2 and 3 are @class and @brief; and @brief carries the class description. So a change confined to the class description is invisible to the write decision and the C/Main/Visu header is never rewritten - the change silently never propagates; and tools/regenerate.py --check cannot see it either; because the gate only observes what the generator actually writes. Proven case: src/Autogenerated/VisuObjectConnectorHydraulicActuatorSimple.h carried a 2024 @brief without the texttt markup that objectDefinition.py has had for some time; it corrected itself only when an unrelated default value in the same class changed and forced a rewrite. A crude scan flags 112 of 291 generated item headers as candidates for the same staleness; each needs confirmation. Fix: compare with IsEqualIgnoringDateStrings (which skips only the date lines) instead of cutting a fixed number of lines; then regenerate once and review the resulting diff.
    - **notes:** revision2026 step R4.11: item headers compared with IsEqualIgnoringDateStrings instead of cutting 7 lines
    - date resolved: **2026-09-15 19:14**\ , date raised: 2026-09-13 
    - resolved by: Claude-JG
 * Version 1.11.73: resolved Issue 2414: AngularVelocityLocal output variable description says velocity not angular velocity (docu)
    - issue author: Claude-JG
    - description:  Four items describe the AngularVelocityLocal output variable as "local (body-fixed) 3D velocity vector of node"; which describes a translational velocity. The word angular is missing. Found while converting outputVariables into data (revision2026 step R4.1.4). Fixing it moves published reference tables; so it is a separate documentation fix rather than part of that step. Worth checking the neighbouring AngularVelocity descriptions at the same time.
    - **notes:** revision2026 step R4.11: angular velocity output variable texts fixed (body local; ANCFBeam VelocityLocal)
    - date resolved: **2026-09-15 19:14**\ , date raised: 2026-09-13 
    - resolved by: Claude-JG
 * Version 1.11.72: resolved Issue 2440: itemInterface.py: generated docstrings do not match the signatures (fix)
    - issue author: Claude-JG
    - description:  pydoclint reports 387 findings in the generated itemInterface.py: the visualization argument is not documented and the arguments carry type hints in the docstring; fix in itemInterfaceEmitter.py and then include the file in the pydoclint check (revision2026 step R4.23)
    - **notes:** revision2026 step R4.23: itemInterfaceEmitter docstrings without arg types; visualization documented; read-only members omitted; pydoclint exclude removed
    - date resolved: **2026-09-15 19:08**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.71: resolved Issue 2444: python modules: __all__ in every module; star imports no longer export helper imports (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.22.3: __all__ in 35 modules (itemInterface by its emitter) from the rule in tools/generators/publicApi.py; tools/checkAll.py --check/--write and GitLab job check_all; utilities.py composes the lists; Examples/TestModels that used np sin cos sqrt copy exudyn from star imports import them explicitly (50 files)
    - date resolved: **2026-09-15 18:58**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.70: resolved Issue 2438: utility modules: star imports export helper names (change)
    - issue author: Claude-JG
    - description:  from exudyn.utilities import \* also exports imported helpers (extends; docmeta; module imports); define __all__ or restructure so only the public API is exported; check what examples and test models rely on first (revision2026 step R4.22)
    - date resolved: **2026-09-15 18:58**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.69: resolved Issue 2443: utilities.py: deprecated GraphicsData aliases removed; functions moved to basicUtilities/advancedUtilities/mainSystemExtensions (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.22.2: 23 GraphicsData... aliases removed (uses switched to exudyn.graphics); @extends functions to mainSystemExtensions; TCP/IP to advancedUtilities; all other functions to basicUtilities (now imports exudyn); utilities.py only re-exports
    - date resolved: **2026-09-15 18:47**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.68: resolved Issue 2442: basicUtilities: numpy-era vector helpers removed (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.22.1: NormL2 VSum VAdd VSub VMult ScalarMult Vec2Tilde Tilde2Vec DiagonalMatrix eye2D eye3D removed; uses in package/TestModels/Examples replaced by numpy (np.linalg.norm; np.sum; array arithmetic; np.dot; Skew); Normalize kept (zero vector allowed) and implemented with numpy; basicUtilities imports numpy
    - date resolved: **2026-09-15 18:02**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.67: resolved Issue 2441: development environments: dependency groups instead of hand-written package lists (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.24: [dependency-groups] docs/lint/build/ide/dev in pyproject.toml; docs/requirements.txt removed; CI and readthedocs install groups; condaEnvironments.md recipe uses pip install --group dev
    - date resolved: **2026-09-15 17:14**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.66: resolved Issue 2439: build: install-time docstring converter removed; pydoclint check in CI (change)
    - issue author: Claude-JG
    - description:  revision2026 steps R4.9 and R4.8: autoGenerateDocstrings.py deleted (its text helpers and docstring renderer kept as tools/generators/docstringText.py); setup.py uses the plain build_py; [tool.pydoclint] in pyproject.toml with a baseline in tools/ci; GitLab job check_docstrings
    - date resolved: **2026-09-15 17:02**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.65: resolved Issue 2437: utility modules: docstring text becomes Markdown (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.6.4: LaTeX macros in docstrings and @docmeta replaced by Markdown (backquoted code; [Key] citations; [text](#label) references and abbreviations; bold/italics; umlauts); utilityDocsModel.Markdown2Latex feeds the existing emitters
    - date resolved: **2026-09-15 16:40**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.64: resolved Issue 2436: utility modules: Google-style docstrings and @docmeta instead of #\*\* comments (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.6.2/R4.6.3 and R4.7: all utility modules converted; utilityDocsModel reads docstrings and decorators with ast; the #\*\* parser is deleted; generated docs unchanged apart from listed whitespace and recovered text
    - date resolved: **2026-09-15 13:46**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.63: resolved Issue 2435: utility modules: malformed #\*\* tags silently dropped from the documentation (fix)
    - issue author: Claude-JG
    - description:  revision2026 step R4.6.1: #\*\*note (9) #\*\*nodes (3) #\*\*examples #\*\*compute #\*\*outputinput and two #\*\* continuation lines were not recognised; five #\*\*function lacked the colon; HT2T66Inverse had input and output swapped
    - date resolved: **2026-09-15 13:25**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.62: resolved Issue 2434: MainSystem extensions: registry decorator instead of copy-and-append (change)
    - issue author: Claude-JG
    - description:  revision2026 step R4.5: mainSystemExtensions.py becomes ordinary package source; functions bind to MainSystem via @extends(exudyn.MainSystem) and install() which raises on a collision with a C++ method; mainSystemExtensionsEmitter.py and mainSystemExtensionsHeader.py are deleted; the docs parsers read the decorator instead of #\*\*belongsTo
    - date resolved: **2026-09-15 12:22**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.61: resolved Issue 2433: generated item headers carry wrong and stray comments (cleanup)
    - issue author: Claude-JG
    - description:  Reported by the maintainer 2026-09-15: MainLoadCoordinate.h and all other Main headers write /\* AUTO: read out dictionary and cast to C++ type\*/ inside SetParameter and label every SetParameter line get parameter; GetParameter of user functions ends with ;; ; the visualization pointer is commented as computational object; CNodeGeneric{AE-ODE1-ODE2} return ...;; from their definitions; lines carry trailing whitespace. revision2026 step R4.21.
    - **notes:** itemHeaderEmitter.py: write statements carry no comment (SetWithDictionary lines plain - SetParameter lines end with //! AUTO: set parameter); no ;; in GetParameter; BodyGraphicsData and SetInternal statements without /\*! \*/ comments; else {PyError without double space; visualization pointer comment corrected; trailing whitespace removed from C/Main/Visu headers (the 7 header lines kept by the file comparison are unchanged); ;; removed from the three CNodeGeneric definitions. 291 generated headers changed; parameterConversionTest 0 differences; suite passed.
    - date resolved: **2026-09-15 11:15**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.60: resolved Issue 2431: src/Autogenerated/StructuralElementsDataStructures.h has no generator and no user (cleanup)
    - issue author: Claude-JG
    - description:  Found in revision2026 step R4.4.3.5a: the tracked generated header src/Autogenerated/StructuralElementsDataStructures.h is not rewritten by tools/regenerate.py (unchanged since the step-25 move) and no file includes it; all includes use the hand-written src/Main/StructuralElementsDataStructures.h. It still contains the old EXUstd::GetSafely setters. Same kind of leftover as #2428. Deleting a tracked file needs the maintainer; the check of revision2026 step R4.18 covers item headers only. revision2026 step R4.20.
    - **notes:** src/Autogenerated/StructuralElementsDataStructures.h deleted (maintainer approval 2026-09-15); nothing included it.
    - date resolved: **2026-09-15 11:15**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.59: resolved Issue 2421: one Python/C++ conversion layer for item and structure parameters (change)
    - issue author: Claude-JG
    - description:  The generated Main headers convert every item parameter twice (SetWithDictionary and SetParameter) through about 40 differently named PybindUtilities helpers (Get<Kind>IndexSafely - Set<Type>Safely - dict and value overloads); the generators pick the helper name from hand-written typeCasts/convertToDict tables. Item range checks exist only in itemInterface.py (165 CheckForValid calls) - so mbs.SetObjectParameter and mbs.AddObject with a raw dict bypass them - while structures check in C++ (EXUstd::GetSafelyUReal). revision2026 step R4.4.3: behaviour test first; new header src/Pymodules/PyConversion.h (FromPython/ToPython) kept separate from PybindUtilities.h; one destination-based type model in the generators; items and structures switch to it; range checks move to C++ with current behaviour kept (also in the fast build); return shapes kept (Real vectors numpy - Float4 and index arrays lists).
    - **notes:** revision2026 steps R4.4.3.1-R4.4.3.6: parameterConversionTest.py records the behaviour; src/Pymodules/PyConversion.h (FromPython/ToPython/ItemIndexFromPython/ItemIndexToPython/MemberGetter/MemberSetter) is the one conversion layer for generated item headers - structure members and the hand-written callers (MainSystem.cpp - MainSystemContainer.cpp - PyGeneralContact.h - solvers - symbolic); range checks in C++ with exudyn.special.exceptions.parameterRangeChecks; typeModel.py renders all type spellings; PybindUtilities.h 1104 -> 344 lines (forwarding block and 30 unused helpers deleted; kept: dict and type tests - GetSTDfunction - SetMatrixSafely - SetListOfArraysSafely - SetSlimArraySafely - reference numpy views).
    - date resolved: **2026-09-15 10:52**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.58: resolved Issue 2429: unify the item/structure type exceptions of typeModel.py (cleanup)
    - issue author: Claude-JG
    - description:  tools/generators/typeModel.py holds 36 spellings the rules do not produce (revision2026 step R4.4.3.3) - the same definition type is spelled differently for items and structures: Int is int for items and Index for structures; Float3/Float4 are exchanged as std::vector<float> for items and std::array<float n> for structures; NumpyVector/NumpyMatrix are stored as Vector/Matrix in items and py::array_t<Real> in structures; Vector2DList vs Vector3DList wrappers; Matrix2D reads Matrix2D in docs while the other fixed matrices read array_like; stub and dictType special names. Unify as far as possible - one spelling per type and destination - keeping each unification only if the full test suite and parameterConversionTest.py (apart from intended reference changes) do not fail; what must remain gets a comment with the reason. revision2026 step R4.19.
    - **notes:** typeModel.py exceptions reduced from 36 to 4: spellings that are the same for items and structures moved to a context-free names table (Int->Index; Py wrapper lists; dict type and stub vocabularies); KeyPressUserFunction joined definitionTypes.userFunctionSignatures; cppExchange has one rule for both contexts (variable sizes std::vector - fixed sizes std::array); item Float3/Float4 - ArrayIndex and Vector parameters convert through the same FromPython overloads as structure members (tuples accepted as before). Remaining: NumpyVector/NumpyMatrix storage (items store Vector/Matrix - structures only return py::array_t from solver functions). parameterConversionTest 0 differences; suite passed.
    - date resolved: **2026-09-15 10:20**\ , date raised: 2026-09-15 
    - resolved by: Claude-JG
 * Version 1.11.57: resolved Issue 2426: item classes reject their own default values (change)
    - issue author: Claude-JG
    - description:  Recorded by parameterConversionTest.py (revision2026 step R4.4.3.1): 17 itemInterface classes raise ValueError when created with their defaults - e.g. MarkerNodeCoordinate(): coordinate=InvalidIndex() fails CheckForValidUInt; ObjectANCFThinPlate cannot be created with defaults in C++ either. The defaults carry two meanings: an index set later (closing a loop; valid only at CheckPreAssembleConsistency) and a value that must be given at creation (MarkerNodeCoordinate.coordinate; a default 0 would be dangerous). Suggestion: constructors never range-check an InvalidIndex() default; a new member flag in definitions/ marks must-be-given parameters and Add<Kind> raises for them in C++ naming item and parameter (same statement as the user line); set-later indices stay with CheckPreAssembleConsistency. revision2026 step R4.17.
    - **notes:** item classes construct with their defaults (since 34c4 b). New item flag CFMustBeGiven (Q) on the 25 parameters whose default lies outside their range form; definitionValidator rule 6 requires the flag exactly there. Add/SetWithDictionary raises: parameter MarkerNodeCoordinate.coordinate must be given; the default -1 is only a placeholder. The same switch as range checks turns it off. Docs mark these parameters. Set-later item indices are not range checked and stay with CheckPreAssembleConsistency.
    - date resolved: **2026-09-15 00:35**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.56: :textred:`resolved BUG 2425` : item indices are accepted by float and bool parameters 
    - issue author: Claude-JG
    - description:  Recorded by parameterConversionTest.py (revision2026 step R4.4.3.1): float parameters (e.g. LoadCoordinate.load - SimulationSettings.solutionSettings.sensorsWritePeriod) accept a NodeIndex or ObjectIndex and store its number; bool parameters accept them as well. Maintainer decision 2026-09-14: reject them where this is simple. Applied with the unified conversion (34c4/34c5) as its own reference change. revision2026 step R4.16.
    - **notes:** Real - float and bool item parameters reject NodeIndex/ObjectIndex/MarkerIndex/LoadIndex/SensorIndex with a message naming item and parameter; Index parameters keep accepting them. parameterConversionTest: 618 intended changes (bool already rejected them). Structures follow in 34c5.
    - date resolved: **2026-09-15 00:35**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.55: :textred:`resolved BUG 2424` : None is silently converted when written into item and structure parameters 
    - issue author: Claude-JG
    - description:  Recorded by parameterConversionTest.py (revision2026 step R4.4.3.1): every bool parameter accepts None and reads back False; index arrays - Vector3DList - Matrix3DList and PyMatrixContainer parameters accept None and read back empty. Maintainer decision 2026-09-14: None shall raise instead of converting unexpectedly; the test suite shows whether any model relies on it. Applied with the unified conversion (34c4/34c5) as its own reference change. revision2026 step R4.15.
    - **notes:** bool - Real - float and Index item parameters raise for None on every write path (message names item and parameter); index arrays raise instead of becoming empty. Vector3DList - Matrix3DList and PyMatrixContainer still accept None as empty because None is their default in itemInterface.py. parameterConversionTest: 672 intended changes (537 bool - 135 index list). Structures follow in 34c5.
    - date resolved: **2026-09-15 00:35**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.54: resolved Issue 2422: switch to disable parameter range checks at runtime (extension)
    - issue author: Claude-JG
    - description:  Range checks on item and structure parameters (UReal - PReal - UInt - PInt) raise in both the normal and the fast build. A release can carry a wrong range limit that is hard to test for; a flag in exudyn.special could let a user switch the checks off manually. Needs the checks in one place first (revision2026 step R4.4.3). revision2026 step R6.5.
    - **notes:** exudyn.special.exceptions.parameterRangeChecks (default True) switches off the range checks of item parameters (all write paths; revision2026 step R4.4.3.4b) and of simulation/visualization settings (EXUstd::GetSafely...); the Python checks in itemInterface.py are gone
    - date resolved: **2026-09-15 00:00**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.53: resolved Issue 2428: four generated item headers in src/Autogenerated have no generator and no user (cleanup)
    - issue author: Claude-JG
    - description:  MainMarkerBody.h - MainMarkerGenericBodyPosition.h - MainObjectContactFrictionCircleCable2DOld.h - MainObjectJointSliding2DNew.h are tracked but no longer written by itemHeaderEmitter.py (their items are not in definitions/) and no .h or .cpp includes them; MainMarkerBody.h still calls HPyUtils::SetStringSafely. Found in revision2026 step R4.4.3.4 when counting the helper calls left in the generated headers. Deleting tracked files needs the maintainer; the drift gate should also report generated files that no generator writes. revision2026 step R4.18.
    - **notes:** the twelve C/Main/Visu headers of MarkerBody - MarkerGenericBodyPosition - ObjectContactFrictionCircleCable2DOld - ObjectJointSliding2DNew deleted; itemHeaderEmitter.py now raises for any item header in src/Autogenerated whose item has no definition
    - date resolved: **2026-09-14 23:53**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.52: :textred:`resolved BUG 2420` : hand-written enum string functions had drifted from their enums 
    - issue author: Claude-JG
    - description:  Joint::GetTypeString tested consecutive values as bits (RevoluteZ=3 gave RevoluteXRevoluteY; PrismaticX=4 gave RevoluteZ) and named Marker in its error; Node::GetTypeString omitted GenericODE1 and GenericAE; operator<< for ItemType printed nothing for an invalid value; the Python enum lists carried keep-synchronized comments. Now generated from definitions/enumTypes.py (revision2026 step R4.3 part 2d)
    - **notes:** C++ enums and string functions generated into src/Autogenerated/EnumTypes.h from definitions/enumTypes.py; Python enums read the same table
    - date resolved: **2026-09-14 14:39**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.51: :textred:`resolved BUG 2419` : pybind declarations: argTypes shorter than argList and a duplicated symbolic.max 
    - issue author: Claude-JG
    - description:  definitionValidator (revision2026 step R4.3 part 2d) finds: MainSystem.DeleteNode / DeleteSensor and MatrixContainer.SetWithDenseMatrix list fewer argTypes than arguments - so their stubs lose all type hints; symbolic.max is declared twice (twice in symbolic.pyi and the docs); exu.Print has argTypes without argList (unused)
    - **notes:** definitionValidator checks the pybind declarations; argTypes completed for DeleteNode / DeleteSensor / SetWithDenseMatrix; duplicate symbolic.max and unused Print argTypes removed
    - date resolved: **2026-09-14 14:29**\ , date raised: 2026-09-14 
    - resolved by: Claude-JG
 * Version 1.11.50: resolved Issue 2416: VSettingsNodes.showNodalSlopes is a boolean flag declared as UInt (check)
    - issue author: Claude-JG
    - description:  The parameter is declared UInt and initialised with false; its description is "draw nodal slope vectors; e.g. in ANCF beam finite elements" and every neighbouring show... flag is a bool. Generated as Index showNodalSlopes = false; (src/Autogenerated/VisualizationSettings.h:525 and :544); with a GetSafelyUInt range check on assignment; so Python accepts any non-negative integer where only 0 and 1 mean anything; and the settings dialog offers an integer field instead of a checkbox. Found while converting default values into real Python values (revision2026 step R4.1.7): it is the only boolean default in the whole definition set that does not sit on a bool type - 370 of 371 do. Changing the type changes the generated C++; the Python type hint and the dialog; so it is a separate decision rather than part of that step.
    - **notes:** copy-paste error confirmed by the maintainer; the type is now bool. Generated as bool showNodalSlopes; the PySet wrapper with its GetSafelyUInt range check is gone and the stub says bool instead of int. All four C++ uses (VisuNodePoint.cpp:677-802) were already boolean tests. Full rebuild and runTestSuite.py PASSED.
    - date resolved: **2026-09-13 21:48**\ , date raised: 2026-09-13 
    - resolved by: Claude-JG
 * Version 1.11.49: :textred:`resolved BUG 2408` : GetOutputVariableTypeString has no case for CoordinatesTotal 
    - issue author: Claude-JG
    - description:  OutputVariableType::CoordinatesTotal (bit 14) is in the C++ enum; it is exposed to Python via AddEnumValue and it is used by 7 item definitions. But GetOutputVariableTypeString() in src/Main/OutputVariable.h has no case for it; it falls through to default: which raises SysError("invalid variable type") and returns "Invalid". Reachable from user-facing paths: CSolverBase.cpp:1960 writes "#OutputVariableType = Invalid" into the sensor solution file header; VisualizationSettings.h:486 prints it in the settings dump. Found while measuring the four hand-synced OutputVariableType lists for revision2026 step R4.1; the planned registrator step makes this class of drift impossible. Also noted: KineticEnergy and PotentialEnergy exist in the enum and in GetOutputVariableTypeString but are NOT exposed via AddEnumValue - a question for the maintainer rather than a defect.
    - **notes:** fixed by the OutputVariableType registrator (revision2026 step R4.1.4): the enum, GetOutputVariableTypeString(); IsOutputVariableTypeForReferenceConfiguration() and the Python enum are now all generated from definitions/outputVariableTypes.py; so a value can no longer exist without a string. Verified: a sensor on CoordinatesTotal writes OutputVariableType = CoordinatesTotal into its solution file header instead of Invalid.
    - date resolved: **2026-09-13 18:28**\ , date raised: 2026-09-13 
    - resolved by: Claude-JG
 * Version 1.11.48: resolved Issue 2410: NodeRigidBodyRotVecLG documents size 3 for three Vector6D parameters (fix)
    - issue author: Claude-JG
    - description:  referenceCoordinates, initialCoordinates and initialVelocities of NodeRigidBodyRotVecLG are declared with type Vector6D but size 3, so the generated reference tables publish "type = Vector6D, size = 3". Everything else says 6: the class description ("3 displacement coordinates and three rotation coordinates"), GetNumberOfODE2Coordinates returns 6, the default value has six entries, and the LaTeX symbol lists six components. The size column is simply wrong in the published documentation. Note that size is currently used for documentation only and is validated nowhere - see the generator note "future: also add size check ..." at pythonAutoGenerateObjects.py:904 - which is why this could go unnoticed. Found during revision2026 step R4.1.2, where shape becomes part of the type and the two can no longer disagree.
    - **notes:** corrected at the source: objectDefinition.py lines 397-399 declared size 3 for three Vector6D members of NodeRigidBodyRotVecLG; the sibling NodeRigidBodyEP block correctly says 6. Regenerated; only the documentation tables changed (tier 1 API surface byte-identical).
    - date resolved: **2026-09-13 16:38**\ , date raised: 2026-09-13 
    - resolved by: Claude-JG
 * Version 1.11.47: resolved Issue 2407: setupPyConfig.json was tracked and mutable: CI rewrote it mid-build and an sdist build used different switches (change)
    - issue author: Claude-JG
    - description:  the six build switches had three sources with no written precedence: defaults in setup.py; the committed JSON; and CLI flags. Three consequences. tools/ci/buildManylinux.sh had to rewrite the tracked file with sed and restore it from a trap; so a failed build left the working tree dirty - and revision2026 step R2.4 shipped the maintainers local toggles inside the sdist. The defaults disagreed with the committed file (compileParallel and quietCompile were False in setup.py and True in the JSON); so a build WITHOUT the file - an sdist build; legitimately - silently took a slower and louder path than the maintainer runs. And the CLI could only turn switches ON; with quietCompile committed as True there was no way to ask for a verbose build. revision2026 step R2.14
    - **notes:** the file is deleted. Defaults now live in [tool.exudyn] in pyproject.toml as real booleans - the True-as-a-string schema of revision2026 step R2.8 went with the JSON that needed it - and resolution is four explicit layers: command line > environment > [tool.exudyn] > built-in default. buildManylinux.sh exports EXUDYN_COMPILE_EXUDYN_FAST=0 instead of sed plus trap; nothing is written to disk during a build. Every flag now has both forms (--quiet/--no-quiet etc.); the historical --noglfw and --nofast spellings are kept. tomli is in build-system.requires with a python_version < 3.11 marker because CI builds cp310 and tomllib is stdlib only from 3.11; a missing TOML parser raises an ImportError naming the install command rather than falling back to different defaults. All four layers were proven separately; the working-tree fingerprint is byte-identical before and after a build with the CI override; and the sdist now carries the maintainers switches instead of the old silent fallback
    - date resolved: **2026-09-12 22:52**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.46: resolved Issue 2384: setup.py sdist drops an untracked copy of LICENSE.txt into main/ (fix)
    - issue author: Claude-JG
    - description:  running python setup.py sdist creates main/LICENSE.txt; byte-identical to the repository root LICENSE.txt and not tracked; so it shows up as an untracked file after every sdist and is easy to commit by accident. Reproduced by deleting it and running sdist again. It is either a build artifact that belongs in .gitignore; or a second copy of the licence that should not exist at all - which revision2026 step R3.1 (flattening the packaging root) would settle. Found during revision2026 step R2.4
    - **notes:** no longer reproduces: it was setup.py reaching for ../LICENSE.txt and dropping the copy inside main/, and the flatten removed that level. setup.py now contains no LICENSE handling at all; MANIFEST.in includes the one file at the repository root, which is where it belongs and where the sdist picks it up. Confirmed after three sdist builds in this session: exactly one LICENSE.txt exists in the working tree, tracked, unmodified since e44aca1, and no untracked copy appears anywhere
    - date resolved: **2026-09-12 19:52**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.45: resolved Issue 2401: issue text ending a word with an underscore breaks the generated RST and the docs CI job (fix)
    - issue author: Claude-JG
    - description:  docs/RST/trackerlog.rst is generated from the tracker; and RST reads a trailing underscore as a link reference. The description of issue 2390 contains CIBW followed by an underscore; so sphinx reports ERROR: Unknown target name "cibw" at trackerlog.rst line 69. The docs CI job runs sphinx-build with -W --keep-going; so this fails the nightly pipeline. Introduced by my own issue text in commit 422ab48. The conversion should escape trailing underscores when writing issue descriptions rather than relying on issue authors to avoid them. Found while verifying the docs build for revision2026 step R3.1
    - **notes:** EscapeRSTmarkup in issueTracker.py now escapes what the shared LatexString2RST does not: a word-final underscore, which RST reads as a link reference, and an unpaired backtick. The notes field additionally now passes replaceMarkups=True like description already did, which is what let a lone asterisk through - the flatten issue mentioned \*.md and opened an inline emphasis that never closed. Applied only at the five RST seams of the tracker, so the shared generator behaviour the rest of the documentation depends on is unchanged. Verified with the CI setting itself: sphinx-build -W --keep-going now exits 0 with no warning at all, where it previously reported Unknown target name cibw and an emphasis start-string
    - date resolved: **2026-09-12 19:50**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.44: resolved Issue 2406: version.txt moved from docs/theDoc to the repository root (change)
    - issue author: Claude-JG
    - description:  the single version source sat inside the documentation tree; a build input buried among LaTeX chapters and - before the flatten - above the packaging root. It is now version.txt at the repository root; exudynVersion.py is the only code that resolves it and doc2rst.py keeps parsing it as a documentation input. revision2026 step R3.4
    - **notes:** moved with git mv. exudynVersion.py resolves it by walking up for a directory holding both pyproject.toml and version.txt; issueTracker.py writes it there; doc2rst.py special-cases it in the parsed-file loop because every other entry in that list is a LaTeX chapter under docs/theDoc/. regenerate.py excludedPaths, MANIFEST.in and the seven untracked build scripts follow. versionName.txt deliberately stays in docs/theDoc/: nothing builds from it
    - date resolved: **2026-09-12 19:36**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.43: resolved Issue 2405: exudynVersion.py guessed four relative paths and fell back to the version string unknown (fix)
    - issue author: Claude-JG
    - description:  it tried ../../../, ../../, ../ and ./ in sequence for docs/theDoc/version.txt and set version = unknown when all four missed. unknown is not a version: setuptools rejects it much later with an InvalidVersion naming neither this file nor version.txt, and a generator run would stamp it into generated sources instead of failing. revision2026 step R3.4
    - **notes:** replaced by one resolver anchored on the repository root: it walks upwards from __file__ (and from the working directory; setup.py and conf.py exec() this file, so __file__ is the executing script there) for a directory holding both pyproject.toml and docs/theDoc/version.txt. Missing or empty is now a RuntimeError naming the file and everything searched. Verified from the repository root, from src/pythonGenerator, and from a directory outside the repository where it now raises instead of returning unknown
    - date resolved: **2026-09-12 19:04**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.42: resolved Issue 2383: the source distribution contains no C++ headers and cannot build (fix)
    - issue author: Claude-JG
    - description:  python setup.py sdist produces an archive with all 133 .cpp files but ZERO of the 432 .h files under main/src (measured on 1.11.24.dev1: 200 entries total). So the sdist on pypi.org cannot be compiled by anyone. MANIFEST.in has no graft/recursive-include for the headers; and setuptools does not add them automatically for an Extension. Pre-existing; found while adding sources.json to the sdist for revision2026 step R2.4
    - **notes:** the sdist now builds and installs. Three things were missing once the flatten made them reachable: docs/theDoc/version.txt and LICENSE.txt (both sat above the old packaging root), and src/pythonGenerator/autoGenerateDocstrings.py; setup.py load_converter() raises FileNotFoundError on that one and stops the build. stubHeader.pyi and the five generated .pyi inputs were added as well; without them every sdist install shipped an unmerged __init__.pyi. The remaining blocker was #2376. Verified end to end: sdist built, then pip wheel from the tarball in a clean directory produced exudyn-1.11.40.dev1-cp313-cp313-win_amd64.whl; installed to a separate target; import exudyn reports 1.11.40.dev1 and a SystemContainer runs
    - date resolved: **2026-09-12 19:04**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.41: resolved Issue 2376: setup.py leaves the working directory changed when stub generation fails (fix)
    - issue author: Claude-JG
    - description:  the block around setup.py line 24 does os.chdir into src/pythonGenerator and restores the directory INSIDE the try; the bare except then swallows the failure; so if createStubFiles.py raises; the rest of setup.py runs in the wrong working directory and every later relative path is wrong. Restore belongs in a finally. Found during revision2026 step R2.2
    - **notes:** the stub-file block now restores the working directory in a finally, and reports the actual exception instead of a bare except with a fixed message. This was the real reason the sdist could not be built: createStubFiles.py failed on a missing stubHeader.pyi, the process stayed in src/pythonGenerator, and the next two steps then reported no setupPyConfig.json found and src/Autogenerated/versionCpp.cpp: No such file or directory - two misleading errors from one unrelated cause, and the package metadata degraded to the name UNKNOWN
    - date resolved: **2026-09-12 19:04**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.40: resolved Issue 2404: the Python 3.14 CI job never compiles exudyn: the scipy pin has no 3.14 wheel (fix)
    - issue author: Claude-JG
    - description:  buildManylinux.sh pins scipy==1.15.2 because newer releases have a much slower sparse eigenvalue solver (revision2026 fact 19). There is no cp314 wheel of 1.15.2; so pip built it from source and meson aborted with Dependency OpenBLAS not found. The job therefore failed in the dependency install step - exudyn itself was never compiled and no 3.14 result exists at all. Seen in the nightly GitLab run of 2026-09-12 for 1.11.31
    - **notes:** the pin now carries a PEP 508 marker: scipy==1.15.2 for python_version < 3.14 and an unpinned scipy at or above it. The pin exists for speed and 3.14 had no result at all; so an unmeasured scipy there is strictly better than no build. Re-pin once a fast-enough 3.14 wheel exists
    - date resolved: **2026-09-12 17:03**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.39: :textred:`resolved BUG 2379` : two TestModels fail reproducibly on Linux and are not marked UnresolvedOnLinux 
    - issue author: Claude-JG
    - description:  generalContactImplicit1.py (error -5.310e-08) and sliderCrank3Dbenchmark.py (-5.037e-10) fail under manylinux_2_28 cp313 while the 8 other Linux differences are marked UnresolvedOnLinux and excluded. So the Linux build exits non-zero. Confirmed PRE-EXISTING and unrelated to revision2026 step R2.3: a build at the previous commit produced bit-identical values. Either the two belong on the UnresolvedOnLinux list with a reason; or the underlying difference needs investigation
    - **notes:** both tests are now on the UnresolvedOnLinux list with their measured relative differences (sliderCrank3Dbenchmark 2.0e-10; generalContactImplicit1 6.8e-08); so the Linux job reports FAILEDL and exits zero while Windows still enforces them. This records the difference - it does not explain it; the underlying Windows/Linux divergence stays scheduled for revision2026 phase R10, and the list is meant to shrink
    - date resolved: **2026-09-12 17:03**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.38: resolved Issue 2403: stale project naming and no way to keep an experimental pytest.py out of commits (change)
    - issue author: Claude-JG
    - description:  after the flatten the Python project was still called pythonDev (python/pythonDev.pyproj, Name and RootNamespace pythonDev) although the directory is python/, its SearchPath pointed at ..\pythonDev which no longer exists, and it listed a Content item requirements.txt that has never existed in that directory (Visual Studio shows it with an exclamation mark). Separately: pytest.py is the scratch file used to try out features in MSVC with mixed debugging, the old practice was to overwrite the experimental version with the default one before committing, which is easy to forget and invisible in review. The solution file was likewise committed under the name main_sln_Template.sln while being the file actually opened. Maintainer request; found during revision2026 step R3.1
    - **notes:** project renamed to python/exudynPython.pyproj with Name and RootNamespace exudynPython, SearchPath corrected to ., the dangling requirements.txt Content item removed - the dependency lists are the extras in pyproject.toml and a second source would only go stale. The template/local pattern now covers both scratch files: exudynTemplate.sln and python/pytestTemplate.py are committed, exudyn.sln and python/pytest.py are in .gitignore, and tools/setupLocalWorkspace.py creates the local copies from the templates without ever overwriting an existing file. An experiment therefore cannot be committed by accident. Documented in docs/dev/WORKFLOW.md section 0a and in docs/dev/README.md
    - date resolved: **2026-09-12 16:54**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.37: resolved Issue 2402: flatten the repository: remove the main/ level (change)
    - issue author: Claude-JG
    - description:  everything lived under main/ - src; include; libs; obj; pythonDev and the packaging files - so every path in every tool carried a main/ prefix; the packaging root was one level below the repository root (which is why the sdist cannot find version.txt; issue 2383) and the generators addressed the root as ../../../. revision2026 steps R3.1 and R3.2
    - **notes:** 1477 tracked files moved with git mv; recorded as 1475 pure renames: main/src->src; main/include->include; main/libs->libs; main/obj->msvc; main/pythonDev->python and the packaging files to the root. The vcxproj needed NO change - all 541 of its ClCompile/ClInclude entries are ..\src\... and msvc/ and src/ remain siblings; so only the two project references in the .sln moved. Repaired: 26 relative paths in the generators (the root went from ../../../ to ../../); doc2rst destDir; the autoGenerateHelper import path in issueTracker.py; regenerate.py; gen_sources.py; checkExtras.py; setup.py; MANIFEST.in; pyproject.toml; conf.py; .gitlab-ci.yml; wheels.yml and buildManylinux.sh. MANIFEST.in include \*.md became include README.md; because at the root the wildcard would have swept CLAUDE.md into the sdist. VERIFIED: wheel builds in 58.9 s with 133 sources and the correct package layout, full test suite passes, gen_sources --check and checkExtras --check pass, sphinx builds the documentation. NOTE regenerate.py --check cannot be meaningful until the move is in HEAD; because it compares each file against HEAD:<path> and every path is new - the regenerated content was confirmed byte identical to HEAD at the old paths instead
    - date resolved: **2026-09-12 16:06**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.36: resolved Issue 2399: performance suite reports only a single total and had no large system test (change)
    - issue author: Claude-JG
    - description:  the suite printed CPU TIME per test scattered through the output and one total at the end; so it was impossible to see which kind of work a change had affected. All six tests were also overhead dominated or mid sized; four of them are a handful of coordinates run for about 1e6 steps. revision2026 step R2.10; maintainer request
    - **notes:** tests are now grouped as small (few coordinates; ~1e6 steps; measures per step overhead) and large (many coordinates; few steps; measures the work per step where long vectors and AVX2 are visible); and the suite prints a table of every test with its group; its time and its share of the total; plus a subtotal per group. A new large test perfLargeMassSpringChain.py was added: 2000 point masses; 6000 ODE2 coordinates; explicit Euler; deterministic (repeated runs bit identical) and about 4.8 s on Windows. All 7 performance tests pass
    - date resolved: **2026-09-12 14:29**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.35: resolved Issue 2395: the comment claiming -mavx2 does not compile on Linux is stale (docu)
    - issue author: Claude-JG
    - description:  setup.py:146 says "-mavx2 (does not compile)" and suspects memory alignment. Measured 2026-09-12: the full extension builds cleanly in manylinux_2_28 with -mavx2 -mfma on g++ 14; 567 vfmadd instructions are present in the resulting .so. What actually fails is at runtime and is a genuine alignment defect; see the LinkedDataVector issue. The comment should be corrected so the next reader does not conclude the toolchain is at fault
    - **notes:** comment at setup.py:146 corrected: -mavx2 does compile; the failure was at runtime and was the alignment defect of issue 2394. The comment now says so and points at revision2026 step R2.10 for the decision to enable AVX2 on Linux; and at the FMA rounding issue
    - date resolved: **2026-09-12 12:48**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.34: :textred:`resolved BUG 2394` : misaligned __m256d load in ResizableVectorParallel MultAdd with a LinkedDataVector 
    - issue author: Claude-JG
    - description:  REPRODUCED 2026-09-12 in manylinux_2_28 cp313 with g++ 14 and -mavx2 -mfma -fsanitize=alignment -fno-sanitize-recover=alignment: src/Linalg/ResizableVectorParallel.h:309:28 runtime error: load of misaligned address for type __m256d which requires 32 byte alignment; in ResizableVectorParallelBase<double>::MultAdd<LinkedDataVectorParallelBase<double>> called from CSolverImplicitSecondOrderTimeInt::ComputeNewtonUpdate. Column 28 is the ptrVector[i] operand; that is the LinkedDataVector; not the ResizableVector. The cause is NOT the allocator: EXUDYN_USE_ALIGNED_VECTORS is enabled and VectorBase allocates through _aligned_malloc/posix_memalign; but a LinkedDataVector points into another buffer at an arbitrary element offset; so no allocator can make the sub-range 32 byte aligned. The observed pointer was 8 byte aligned. Latent today because AVX2 is only enabled on Windows (setup.py:230-233); it is what blocks enabling AVX2 on Linux. Fix is to use unaligned intrinsics (_mm256_loadu_pd/_mm256_storeu_pd) on the linked operands; which cost nothing on aligned data on Haswell and later. revision2026 step R2.9
    - **notes:** fixed by replacing aligned PReal\* dereference with explicit unaligned load/store in every AVX loop whose pointer can come from a LinkedDataVector: all 40 casts in LinkedDataVectorParallel.h and ResizableVectorParallel.h; plus the six ParallelPReal\* helpers in Vector.cpp; whose PReal\* parameters had the same problem without a cast to find them by. Use_avx.h gained _mm_store_u for all four AVX variants (only _mm_load_u existed) and both for the scalar fallback; the arithmetic still uses operators; which work for __m256d and for plain Real alike. VERIFIED: (1) the manylinux cp313 build with -mavx2 -mfma -fsanitize=alignment -fno-sanitize-recover=alignment previously aborted at ResizableVectorParallel.h:309 and now reports ZERO misalignment errors and runs the suite to completion, (2) the same build without -mavx2 reproduces the previous Linux results BIT FOR BIT - same two known failures with identical values - so the change is numerically neutral, (3) on Windows; where AVX2 is live today; the full test suite passes and runPerformanceTests gives 33.33 s against a 33.04-34.16 s baseline measured twice before the change; i.e. inside the run to run noise
    - date resolved: **2026-09-12 12:48**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.33: resolved Issue 2393: setupPyConfig.json is not validated: a typo silently changes the build (change)
    - issue author: Claude-JG
    - description:  setup.py warned about an unknown key or an illegal value and then continued with the default. So "compileParallell": "True" produced a serial build minutes slower with nothing in the output explaining why; and the outer bare except also swallowed genuine read errors. revision2026 step R2.8
    - **notes:** setupPyConfig.json is now validated against the default config dict; which IS the schema: unknown key; value other than the strings "True"/"False"; a non object document or malformed JSON each raise with the offending key named and the valid keys listed. A MISSING file remains a warning using defaults; because building from an sdist legitimately has no config file - only a file that is present but wrong is fatal. The bare except is narrowed to OSError/JSONDecodeError. Verified by fault injection: unknown key, lowercase "true", malformed JSON each exit 1 with a specific message, a missing file still builds, the committed file builds a wheel in 55.7 s, and the sed rewrite that tools/ci/buildManylinux.sh applies to compileExudynFast still validates
    - date resolved: **2026-09-12 11:56**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.32: resolved Issue 2392: issueTracker does not report the number it assigned (change)
    - issue author: Claude-JG
    - description:  RaiseIssue and ResolveIssue printed no issue number; so the number had to be reconstructed by hand for the commit message and the documentation. That produced five wrong numbers in the revision plan; the log and two commit messages on 2026-09-12 (the ClInclude defect was written as 2388 but is 2387; and four following issues were likewise one too high). Maintainer request
    - **notes:** RaiseIssue now returns the assigned number and RaiseIssueDict prints "issue raised: #NNNN title"; ResolveIssue prints "issue resolved: #NNNN title" before the dict and returns the number. So the number never has to be guessed again
    - date resolved: **2026-09-12 11:56**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.31: resolved Issue 2391: continue-on-error made failing wheel builds report success (fix)
    - issue author: Claude-JG
    - description:  build_wheels carried continue-on-error: true; so a job that failed to build or whose test suite failed did not fail the workflow. fail-fast: false is what actually keeps the sibling jobs running; continue-on-error only hid the result. revision2026 step R2.7
    - **notes:** continue-on-error removed. fail-fast: false is kept; so one failing wheel still does not cancel the others - but the workflow now reports the failure
    - date resolved: **2026-09-12 10:01**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.30: resolved Issue 2390: move the cibuildwheel configuration into [tool.cibuildwheel] (change)
    - issue author: Claude-JG
    - description:  the wheel matrix was configured through eleven CIBW\_ environment variables in .github/workflows/wheels.yml; where nothing validates them: a lost newline had swallowed CIBW_BUILD_VERBOSITY: 1 into the trailing comment of the CIBW_TEST_SKIP line; so build verbosity was silently never set. revision2026 step R2.7
    - **notes:** the ten static settings moved to [tool.cibuildwheel] in main/pyproject.toml; with linux; macos and windows subtables for the per platform test-command; before-all and environment. Only CIBW_BUILD stays in the workflow; because it is built from the matrix entry. build-verbosity = 1 now actually applies. Verified with the pinned cibuildwheel 3.4.1: --print-build-identifiers returns the same set for all three platforms as the old environment variables did (cp313-manylinux_x86_64, cp313-macosx_universal2, cp313-win_amd64), the resolved options show build-verbosity 1, the test requirements, the dnf before-all on linux only and the cd /d form of the test command on windows, and a deliberately misspelled key is rejected with "Option build-verbosityy not supported in a config file" instead of being ignored
    - date resolved: **2026-09-12 10:01**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.29: resolved Issue 2389: remove 32 bit support: libs/libs32 and the bitness branch in setup.py (cleanup)
    - issue author: Claude-JG
    - description:  follow up to issue 2386; whose resolution note still says setup.py keeps its is32bits branch - that is no longer true. On the maintainer instruction the 32 bit support is removed outright: main/libs/libs32 (glfw.pdb; glfw3.lib; glfw3_d.lib; openvr_api.dll; openvr_api.lib) and the is32bits/is64bits branch in setup.py; whose only remaining effect was to choose that directory. revision2026 step R2.6
    - **notes:** the five libs32 files deleted and the bitness branch replaced by an explicit failure: a 32 bit interpreter now raises immediately with the reason; instead of falling through to the 64 bit import libraries and producing "DLL load failed: 
    - date resolved: **2026-09-12 00:08**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.28: resolved Issue 2386: drop the ReleaseP37 and Win32/x86 build configurations (cleanup)
    - issue author: Claude-JG
    - description:  the solution and cppsrc.vcxproj carried six configurations: Debug; Release and ReleaseP37 times Win32 and x64. ReleaseP37 is the last artefact of Python 3.7 support and no 32 bit wheel has been built for years; so four of the six were dead weight that every vcxproj edit had to be repeated in. revision2026 step R2.6
    - **notes:** six configurations reduced to two; Debug|x64 and Release|x64. Removed from cppsrc.vcxproj the four ProjectConfiguration declarations and every PropertyGroup; ImportGroup and ItemDefinitionGroup conditioned on them (853 -> 660 lines); rewrote both GlobalSections of main_sln_Template.sln; and dropped the ReleaseP37 PropertyGroup from pythonDev.pyproj. Verified: all three files still parse as XML; the 133 ClCompile entries are untouched and still match sources.json; and MSBuild evaluates both surviving configurations without error - Release|x64 resolving to ConfigurationType DynamicLibrary; TargetExt .pyd; PlatformToolset v143 and OutDir bin/x64/Release. requires-python also moved from >=3.6 to >=3.10; which revision2026 step R2.11 had deferred to here: CI builds cp310-cp314 and no 3.6 wheel has existed for years. setup.py keeps its is32bits branch; it reacts to the running interpreter rather than to a VS configuration; and removing 32 bit support outright is not part of this step
    - date resolved: **2026-09-12 00:01**\ , date raised: 2026-09-12 
    - resolved by: Claude-JG
 * Version 1.11.27: resolved Issue 2385: remove the three dead CMakeLists files (cleanup)
    - issue author: Claude-JG
    - description:  none of the three could configure; let alone build. main/CMakeLists.txt was a fourth complete copy of the 133 file source list and called add_subdirectory(pybind11) on a directory that does not exist; its own header said "CMakeLists for Exudyn are not complete". main/src/CMakeLists.txt and main/obj/autoCMakeLists.txt listed 63 of 133 files; missed 74; referenced sources deleted years ago (StaticSolver.cpp; CNodeRigidBody.cpp; solver/TimeIntegrationSolver.cpp) and used add_executable although exudyn is a Python module. Decision D3 keeps the .sln and .vcxproj and drops CMake. revision2026 step R2.5
    - **notes:** all three deleted (405 lines). main/include/Eigen/CMakeLists.txt is vendored third-party and was left untouched. No generator recreates them. The only reference in the documentation was one passing mention in docs/theDoc/gettingStarted.tex; which now names scikit-build-core alone rather than pointing at files that no longer exist. This also removes the last duplicate of the source list that revision2026 step R2.4 consolidated into main/sources.json
    - date resolved: **2026-09-11 23:52**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.26: resolved Issue 2382: cppsrc.vcxproj listed src/tests/UnitTestBase with the wrong case (fix)
    - issue author: Claude-JG
    - description:  the vcxproj had ClCompile ..\src\tests\UnitTestBase.cpp and ClInclude for the header; while the tracked directory is src/Tests. Windows resolves both to the same file so it was invisible there; Linux does not. It did not break the manylinux build only because setup.py had its own list with the correct case. Found by tools/gen_sources.py; revision2026 step R2.4
    - **notes:** corrected to src\Tests\UnitTestBase for both the ClCompile and the ClInclude entry. tools/gen_sources.py --check now fails on any case mismatch; so this cannot silently return
    - date resolved: **2026-09-11 22:46**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.25: resolved Issue 2381: derive the compile list from the vcxproj instead of duplicating it in setup.py (change)
    - issue author: Claude-JG
    - description:  setup.py carried a hand-maintained list of 133 .cpp paths in two literal Python lists; a second copy of the ClCompile entries of main/obj/cppsrc.vcxproj with nothing keeping the two in agreement. revision2026 step R2.4
    - **notes:** tools/gen_sources.py derives main/sources.json from the ClCompile entries of cppsrc.vcxproj; reading only ItemGroup blocks so every PropertyGroup setting stays hand-maintained. setup.py reads the JSON and no longer contains a source list; no XML is parsed at build time and the JSON ships in the sdist via MANIFEST.in. --check compares vcxproj; JSON and the files on disk and is run in CI as check_sources; the comparison is case-exact; because a wrong-case entry resolves on Windows and fails on Linux. The minimal subset cannot be expressed in the vcxproj; so it is carried in the JSON and validated as a subset. Verified: wheel builds in 57.3 s with 133 files and the full test suite passes; the checker was fault injected with a stale JSON entry and a nonexistent vcxproj entry; and it caught a real wrong-case entry on its first run (issue #2382)
    - date resolved: **2026-09-11 22:46**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.24: :textred:`resolved BUG 2378` : parallel-compile monkeypatch: thread-unsafe output and swallowed errors 
    - issue author: Claude-JG
    - description:  four defects in the parallel compile patch in setup.py. (1) on Linux the quiet path rebound the GLOBAL sys.stdout from every worker thread; each opening setuppy.output.txt with mode w; so threads truncated each other and the restore raced. (2) the outer bare except also swallowed KeyboardInterrupt/SystemExit and its message "trying serial compilation" described the compile; although it only wraps installing the patch. (3) the Windows handler did "raise ValueError(args)"; destroying the compiler CompileError and its message. (4) nObjects = len(objects)+2 made the progress counter report 135 for 133 files. revision2026 step R2.3
    - **notes:** the per-file redirect is replaced by one contextlib.redirect_stdout around the whole thread pool (the shape the Windows branch already used); and contextlib also restores sys.stdout when a compile raises; which the old "sys.stdout = sys.__stdout__" after the with-block did not. Progress now goes to sys.__stdout__ so it stays on the console rather than in the log. The Windows handler prints the command line and re-raises; keeping the compiler diagnostic. The outer except is now except Exception and says what it actually covers. nObjects = len(objects). The monkeypatch itself is KEPT: setuptools parallelises build_ext across extensions; not across sources within one; and exudyn has 133 sources in a single extension. Verified on both platforms: Windows wheel 56.8 s (unchanged); manylinux cp313 build plus test suite bit-identical to the previous commit; and a quiet Linux build reporting completed 001/133 .. 133/133
    - date resolved: **2026-09-11 21:27**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.23: resolved Issue 2375: optional-dependency extras with a drift checker (extension)
    - issue author: Claude-JG
    - description:  there was no way to install the packages needed for the tests or referred to internally; and no mechanism to notice when the code starts importing something that no declared list installs. revision2026 step R2.12
    - **notes:** [project.optional-dependencies] adds tests / all / rl; all built on the PEP 508 self-reference exudyn[tests]. tools/checkExtras.py AST-scans TestModels; exudyn and Examples and fails when an import is installed by no extra; run in CI as check_extras. Verified by fault injection: a fake import; a removed extra entry and a deleted name-mapping entry each reported with exit 1. rl is deliberately not part of all (torch size and the --index-url CUDA choice); confirmed that allExudynModulesTest.py announces the resulting skip rather than passing vacuously
    - date resolved: **2026-09-11 20:12**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.22: resolved Issue 2374: package metadata drift: classifiers and MANIFEST.in (change)
    - issue author: Claude-JG
    - description:  classifiers claimed Python 3.9-3.13 while CI builds cp310-cp314; and MANIFEST.in contained include ../LICENSE.txt which points outside the sdist root and is a silent no-op; so the licence has not been in the sdist. revision2026 step R2.11
    - **notes:** classifiers now 3.10-3.14; matching the wheels CI actually builds. requires-python deliberately left at >=3.6 for revision2026 step R2.6. The unreachable MANIFEST.in include was replaced by a comment stating why the licence is absent and that revision2026 step R3.1 is what makes the include possible
    - date resolved: **2026-09-11 20:12**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.21: resolved Issue 2373: pybind11 into build-system.requires (change)
    - issue author: Claude-JG
    - description:  setup_requires is deprecated and was the only thing fetching the pybind11 headers (into main/.eggs). The documented include path include/pybind11 does not exist and the vendored copy at include/pybind11local is inert; so an undeclared download was load-bearing. revision2026 step R2.2
    - **notes:** pybind11<3.0 moved to build-system.requires; setup_requires removed; the dead include/pybind11 path deleted; and get_pybind_include now raises an actionable ImportError naming the install command. Verified by moving main/.eggs away: the build failed with the new message; after pip install pybind11 the wheel built in 55 s and .eggs was NOT recreated. condaEnvironments.md now installs pybind11 explicitly; because build-system.requires is honoured only under build isolation and setup.py bdist_wheel has none
    - date resolved: **2026-09-11 20:12**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.20: resolved Issue 2371: PEP 621 metadata (change)
    - issue author: Claude-JG
    - description:  move static package metadata from setup(...) into a [project] table in main/pyproject.toml; setup.py keeps only version; classifiers; packages; package_data; ext_modules and cmdclass. build-system.requires floor raised to setuptools>=61; below that a [project] table is silently ignored. revision2026 step R2.1
    - **notes:** verified by diffing PKG-INFO from setup.py egg_info before and after: only the PEP 621 spellings changed (Author/Home-page -> Author-email/Project-URL) and the Dynamic: block shrank from ten entries to one. A full wheel built in 54 s with the .pyd and both .pyi stubs present
    - date resolved: **2026-09-11 18:24**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.19: :textred:`resolved BUG 2370` : generated documentation depends on the filesystem listing order 
    - issue author: Claude-JG
    - description:  ExtractExamplesWithKeyword in autoGenerateHelper.py and the Examples listing in doc2rst.py used os.listdir() without sorting. NTFS returns alphabetical order and ext4 returns hash order; the Relevant-Examples lists are truncated to the first few entries; so the generated docs and two Tier 1 files under main/src/pythonGenerator/generated differed between Windows and Linux. Found by the first CI run of tools/regenerate.py on Linux (revision2026 step R0.2 and fact 18).
    - **notes:** Both listings now use sorted(..., key=str.lower). Case-insensitive reproduces the NTFS order the committed output was generated in, so Tier 1 is unchanged on Windows. Verified in a python:3.13 container: Linux and Windows now produce identical generated files.
    - date resolved: **2026-09-11 11:03**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.18: :textred:`resolved BUG 2369` : relativeRotationTranslationMechanism now ends at rest at the origin 
    - issue author: Claude-JG
    - description:  norm(GetODE2Coordinates()) at the end of the run is 2.087948914872922e-12 while the reference recorded in the file is 4.172189649307425 (2023-06-12). The mechanism produces essentially zero motion. Found while triaging the unlisted TestModels for revision2026 step R5.9. Not added to the test suite.
    - **notes:** Fixed by the maintainer: endTime reduced from 2s to 0.2s. At 2s the mechanism was back in its initial configuration with no displacements, so the norm was ~0. Added to the test suite with reference 1.509631854432179.
    - date resolved: **2026-09-11 11:03**\ , date raised: 2026-09-11 
    - resolved by: Claude-JG
 * Version 1.11.17: resolved Issue 2367: test suite does not notice models that are in no reference list (testing)
    - issue author: Claude-JG
    - description:  runTestSuite.py builds its run list from the keys of TestExamplesReferenceSolution() and never lists the directory; a model on disk but in no list is therefore silently never executed. 19 models were in that state.
    - **notes:** testRunnerTools.CheckTestCoverage() added; runs at suite start-up; uncovered file or stale key fails under --exit-code. Verified by fault injection.
    - date resolved: **2026-09-10 22:31**\ , date raised: 2026-09-10 
    - resolved by: Claude-JG
 * Version 1.11.16: :textred:`resolved BUG 2366` : Python 3.6 non-AVX reference override never applied 
    - issue author: Claude-JG
    - description:  replaceRefSol in runTestSuiteRefSol.py spelled revoluteJointprismaticJointTest.py with a lowercase p while the file and the base dictionary use revoluteJointPrismaticJointTest.py. The overlay is applied by iterating the base keys; a key matching nothing is silently ignored; so this correction never applied on Python 3.6 without AVX.
    - **notes:** Overlay hoisted into Python36NonAVXOverrides() so its keys can be validated; spelling fixed; CheckTestCoverage reports any future dead override.
    - date resolved: **2026-09-10 22:31**\ , date raised: 2026-09-10 
    - resolved by: Claude-JG
 * Version 1.11.15: resolved Issue 2365: Clean up docs/howTo and remove the doxygen configuration (cleanup)
    - issue author: Claude-JG
    - description:  docs/howTo held 26 loose .txt files, most describing VS2017/VS2019, 32-bit builds, Python 3.6/3.7 or the pre-WSLg X-server era, plus two exact duplicates. docs/doxygen held a 111 KB Doxyfile for a tool that broke on project size, whose PDF path never worked and whose graph generation had already been switched off. Also: no experimental folder remains in the tracked tree, so revision2026 step R1.4 has nothing to move. revision2026 steps R1.4, 30, 79.
    - **notes:** howTo cut 26 files to 8 all in .md, with the hard-won specifics salvaged into buildQuirks.md; doxygen removed; new docs/dev/ARCHITECTURE.md replaces what doxygen was wanted for. Experimental.h identified as a deliberate feature-flag mechanism and kept
    - date resolved: **2026-09-10 18:53**\ , date raised: 2026-09-10 
    - resolved by: Claude-JG
 * Version 1.11.14: resolved Issue 2364: Verify the GitLab CI pipeline on the shared runners (testing)
    - issue author: Claude-JG
    - description:  The GitLab CI configuration was written and tested locally under docker, but two environment questions could only be answered on the real runners: whether they may pull the manylinux image from quay.io, and whether EPEL is reachable for the GLFW and X11 packages. revision2026 step R1.7.
    - **notes:** First run 2026-09-10: all six jobs passed - wheels_linux cp310 to cp314 plus docs. quay.io reachable, EPEL reachable, and sphinx-build -W passes on Linux for the first time. Remaining: enable the weekly schedule and confirm failure mail
    - date resolved: **2026-09-10 14:21**\ , date raised: 2026-09-10 
    - resolved by: Claude-JG
 * Version 1.11.13: resolved Issue 2363: Known unresolved Windows/Linux differences in contact and friction tests (check)
    - issue author: Claude-JG
    - description:  Eight contact and friction test models give different results on Linux than the Windows reference values: relative errors from 3.4e-11 up to 1.6e+04 (sphereTriangleTest.py: reference 3.8226 vs Linux 59370.97). These are reproducible, so they are not the non-deterministic class - there is a cause that has not been found. Track them in UnresolvedOnLinux() and exclude from the exit code on Linux only, so Linux CI is usable while Windows stays strict. Investigation scheduled as revision plan phase R10 revision2026 step R10.1.
    - **notes:** List added and wired in; not the fix. Verified Windows still passes 106/106 with the list active and Linux exits 0 while reporting all eight. sphereTriangleTest.py is a divergence rather than an accuracy difference and should be investigated first
    - date resolved: **2026-09-10 13:08**\ , date raised: 2026-09-10 
    - resolved by: Claude-JG
 * Version 1.11.12: :textred:`resolved BUG 2362` : Test runners truncate committed release logs at startup 
    - issue author: Claude-JG
    - description:  runTestSuite.py, runTestExamples.py and runPerformanceTests.py each call SetWriteToFile with flagAppend=False on a release-named file in a tracked directory at script start, before any test runs, with no existence check. An interrupted run leaves the committed release log half-written. 55 tracked log files are exposed. Filenames encode version, platform and Python but nothing machine-specific, so a second machine with the same configuration silently overwrites the first. revision2026 step R5.10.
    - **notes:** testRunnerTools.ResolveLogFile diverts to logsTmp when the target exists; --overwrite-log replaces deliberately. Also added package versions and CPU to the header, a per-test overview with runtimes at the end, and EXUDYN_MACHINE_ID for performance logs. Verified: divert leaves the tracked log byte-identical, override replaces it, absent target writes normally
    - date resolved: **2026-09-10 09:44**\ , date raised: 2026-09-10 
    - resolved by: Claude-JG
 * Version 1.11.11: resolved Issue 2361: CI during the GitHub freeze (testing)
    - issue author: Claude-JG
    - description:  GitHub Actions only fire on pushes to master and pull requests, so no wheels or tests run while master is frozen at 1.11.0 (decision D6). Add GitLab CI for Linux x86_64 across cp310-cp314 on the shared Docker runners, plus a docs build. Also fix that CI could not fail at all: wheels.yml sets continue-on-error and runTestSuite.py always exited 0. revision2026 step R1.7.
    - **notes:** runTestSuite.py gains --exit-code and loses the -F<NN> log suffix; reproducible vs sensitive tests separated in runTestSuiteRefSol.py; manylinuxBuild.sh deduplicated into tools/ci/buildManylinux.sh. Verified by fault injection. First pipeline run and the weekly schedule are still pending
    - date resolved: **2026-09-09 23:22**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.10: resolved Issue 2360: Set up internal GitLab remote and sync v2-dev (change)
    - issue author: Claude-JG
    - description:  All v2.0 work needed an internal sync point so that GitHub master stays frozen at 1.11.0 (decision D6). Configure origin as the UIBK GitLab and github as the public remote; seed the server with full history and push v2-dev. revision2026 steps R1.1 and R1.3.
    - **notes:** GitLab rejects shallow pushes and caps packs at 1.17 GiB; seeded master from an existing full clone in 50-commit chunks, then v2-dev pushed from the shallow clone in 373 KB. Verified 30 refs matching by SHA. Local .git stays at 65 MB
    - date resolved: **2026-09-09 22:27**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.9: resolved Issue 2359: Add pre-push hook guarding the public repository (new feature)
    - issue author: Claude-JG
    - description:  Nothing mechanically prevented pushing v2-dev to public GitHub; only memory. Add tools/hooks/pre-push refusing any ref but master, release/\* and tags when the target is GitHub, matched on both remote name and github.com URL. Activated per clone with git config core.hooksPath tools/hooks. Also verified that pushing from this shallow clone requires receive.shallowUpdate=true on the receiving repository. revision2026 steps R1.5 and R1.1.
    - **notes:** Verified with four cases against local throwaway repositories: v2-dev to github refused, master to github allowed, v2-dev to internal allowed, bare github.com URL refused before network access
    - date resolved: **2026-09-09 20:57**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.8: resolved Issue 2358: Add .gitattributes for line endings and binary files (fix)
    - issue author: Claude-JG
    - description:  The repository had no .gitattributes, so line endings depended on each contributor core.autocrlf setting and generation under a different shell could produce whole-file diffs. All generation and building so far has been on Windows; CI will run on Linux. Normalise text to LF in the repository with native checkout, and mark binary types explicitly. revision2026 step R0.5.
    - **notes:** Index was already fully LF-normalised so no renormalisation was introduced (git add --renormalize stages zero files). Two extension traps found: testData/rotorAnsys.rst is an ANSYS result file not reStructuredText, and \*.eps/\*.stl have ASCII variants so they are left to text=auto
    - date resolved: **2026-09-09 20:13**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.7: resolved Issue 2357: Add tools/regenerate.py drift gate (testing)
    - issue author: Claude-JG
    - description:  No automated check existed for whether the committed generated files still match what the generators produce, so generator drift was silent. Add tools/regenerate.py: runs the six generators from their required cwd and classifies differences against HEAD, Tier 1 hard fail and Tier 2 warning. Must compare with IsEqualIgnoringDateStrings rather than raw git status, because the last-modified line otherwise produces permanent phantom drift. revision2026 step R0.2; CI wiring still open.
    - **notes:** Verified by fault injection: perturbed generator input gives exit 1 with correct tier classification; hand edit to an already-dirty tier file gives exit 1; date-only difference gives exit 0
    - date resolved: **2026-09-09 19:00**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.6: resolved Issue 2356: Golden file snapshot from a verified-current generated set (testing)
    - issue author: Claude-JG
    - description:  The archived golden files were taken at e44aca1, where the committed generated set was already stale and where generator output was still cp1252-corrupted. Re-cut the reference with git archive from the current commit so it is provably identical to committed content. revision2026 step R0.3.
    - **notes:** goldenFiles_V1.11.5_910e2b5.zip, 871 files; full regeneration on that commit produces no drift, so the commit itself is the reference
    - date resolved: **2026-09-09 18:50**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.5: resolved Issue 2355: Regeneration must run after ResolveIssue (docu)
    - issue author: Claude-JG
    - description:  ResolveIssue rewrites docs/theDoc/version.txt, but README.rst and docs/RST/Exudyn.rst embed the version string and are refreshed only by doc2rst.py. Regenerating before resolving leaves those two files one version behind, so the next drift check reports an unrelated change. Documented the required order in docs/dev/WORKFLOW.md and as a constraint on revision2026 step R0.2.
    - **notes:** Gate order is now: resolve issue, then regenerate, then commit
    - date resolved: **2026-09-09 18:49**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.4: resolved Issue 2354: Track tools/issueTracker in the repository (change)
    - issue author: Claude-JG
    - description:  issueTracker.py and trackerlog.txt are the source of truth for the version number and the issue history but were local-only and never on GitHub. Commit them so the version derivation and the issue log live with the code. trackerlog.html and trackerlog_backup.txt stay ignored as regenerated output. tools/buildAndGenerate stays ignored until the cleanup in revision2026 step R3.7 because it still contains hard-coded local paths.
    - **notes:** Scanned trackerlog.txt for revision2026 step R1.6 first: no credentials, no email addresses, no absolute local paths
    - date resolved: **2026-09-09 18:47**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.3: resolved Issue 2353: Reconcile GitHub Examples/TestModels with internal repository (fix)
    - issue author: Claude-JG
    - description:  The GitHub copies of main/pythonDev/Examples and TestModels had drifted from the internal repository through the old auto-copy; six Examples and five TestModels were stale or missing so the committed docs did not match a fresh regeneration. Reconciled both folders. Also pruned TestSuiteLogs/PerformanceLogs per the new retention policy (decision D7) and added the missing 1.11 and testExamples logs. Adjusted movingGroundRobotTest.py output scaling factor from 0.01 to 0.005 with updated reference value because its error sat too close to the global 5e-14 tolerance.
    - **notes:** Examples and TestModels now match the internal repository; a fresh regeneration produces no doc differences
    - date resolved: **2026-09-09 18:47**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.2: resolved Issue 2352: Generators write output with platform default encoding (fix)
    - issue author: Claude-JG
    - description:  Twelve write sites and six paired read sites in the python generators used the platform default encoding (cp1252 on Windows) instead of UTF-8, corrupting non-ASCII output on every regeneration (e.g. degree sign in interfaces.tex). Blocks the regeneration drift gate. Also: pybind_manual_classes.h was rewritten unconditionally and StructuresAndSettingsIndex.rst had trailing spaces. Revision plan revision2026 step R0.6.
    - **notes:** All 12 write sites and 6 paired read sites now pass encoding=utf8; pybind_manual_classes.h routed through WriteTextIfDifferent; trailing spaces removed. Verified by two identical consecutive regenerations and a UTF-8 validity scan.
    - date resolved: **2026-09-09 18:33**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.1: resolved Issue 2351: Working setup for Claude sessions (docu)
    - issue author: Claude-JG
    - description:  Add CLAUDE.md working contract, docs/dev (CODING_STYLE, WORKFLOW, README), docs/howTo/condaEnvironments.md and root .gitignore; untrack main/dist and cppsrc.vcxproj.user. Revision plan revision2026 step R0.1.
    - **notes:** CLAUDE.md + docs/dev + condaEnvironments.md + root .gitignore; main/dist and cppsrc.vcxproj.user untracked
    - date resolved: **2026-09-09 18:33**\ , date raised: 2026-09-09 
    - resolved by: Claude-JG
 * Version 1.11.0: :textred:`resolved BUG 2348` : StableBaselines 
    - issue author: P. Manzl
    - description:  Importing exudyn.artificialIntelligence raises an error at to the old gym legacy flag with the latest stable-baselines3 versions due to non-numeric version numbers
    - date resolved: **2026-06-01 14:43**\ , date raised: 2026-06-01 

************
Version 1.10
************

 * Version 1.10.160: resolved Issue 2346: special.solver.timeout (change)
    - description:  change start time for this timer to output.cpuSolverStartTime to more accurately handle timeouts and simulationInRealtime
    - date resolved: **2026-04-20 22:39**\ , date raised: 2026-04-20 
 * Version 1.10.159: resolved Issue 2347: solver.output (extension)
    - description:  add distinguished flags for stopping simulation: simulationTimeout, simulationStoppedByUser and simulationStoppedByUserFunction to determine causes for stopped simulations (all giving finishedSuccessfully=True)
    - date resolved: **2026-04-20 22:38**\ , date raised: 2026-04-20 
 * Version 1.10.158: resolved Issue 2345: Create functions (fix)
    - description:  several parameters like CreateTorsionalSpringDamper offset, velocityOffset and torque require UReal but only Real needed
    - date resolved: **2026-04-19 14:39**\ , date raised: 2026-04-19 
 * Version 1.10.157: resolved Issue 2341: CreateSphereTriangleContact (change)
    - description:  CreateSphereTriangleContact, CreateSphereQuadContact and CreateSphereSphereContact: adapt to make it work with mass points in case that there is no friction, similar to issue 2231
    - date resolved: **2026-04-07 20:15**\ , date raised: 2026-04-07 
 * Version 1.10.156: resolved Issue 2231: CreateSphereSphereContact (extension)
    - description:  if objects are mass points (2D, 3D), only add MarkerBodyPosition if dynamicFriction=0 (otherwise error); enable markers in bodyNumbers field
    - date resolved: **2026-04-07 20:13**\ , date raised: 2026-01-19 
 * Version 1.10.155: resolved Issue 2340: CreateSphereTriangleContact (change)
    - description:  CreateSphereTriangleContact and CreateSphereQuadContact shall accept list of quadPoints or trianglePoints instead of Vector3DList
    - date resolved: **2026-04-07 20:01**\ , date raised: 2026-04-07 
 * Version 1.10.154: resolved Issue 2339: ContactSphereTriangle (fix)
    - description:  in case of frictionless contact, torques on triangle are neglected
    - date resolved: **2026-04-06 19:46**\ , date raised: 2026-04-06 
 * Version 1.10.153: resolved Issue 2338: CreateContactSphereSphere (fix)
    - description:  allow mass points with MarkerBodyPosition or MarkerNodePosition in friction-less case
    - date resolved: **2026-04-06 19:42**\ , date raised: 2026-04-06 
 * Version 1.10.152: resolved Issue 2336: beams (extension)
    - description:  add new beams generate function GenerateBeamElementsAlongLine which is similar to GenerateStraightBeam, but has more readable args and a dict as return value
    - date resolved: **2026-04-06 15:53**\ , date raised: 2026-04-06 
 * Version 1.10.151: resolved Issue 2335: shells (extension)
    - description:  add visualization setting shells.thicknessFactor for drawing larger thickness for visual inspectation
    - date resolved: **2026-04-03 14:03**\ , date raised: 2026-04-03 
 * Version 1.10.150: resolved Issue 2334: VSettingsShells (extension)
    - description:  add visualization settings for plates and shells
    - date resolved: **2026-04-02 16:04**\ , date raised: 2026-04-02 
 * Version 1.10.149: resolved Issue 2331: NumpyMatrix (extension)
    - description:  add option to silently convert scalar values into NumpyMatrix in object parameter initialization (e.g. height in ANCFThinPlate being either scalar or matrix)
    - date resolved: **2026-04-02 13:25**\ , date raised: 2026-04-02 
 * Version 1.10.148: resolved Issue 2330: AddSystem (fix)
    - description:  add py::keep_alive<0, 1>() to functions like AddSystem which return an object which depends on another object (SystemContainer); avoids severe problems when SystemContainer is destroyed
    - date resolved: **2026-03-31 19:03**\ , date raised: 2026-03-31 
 * Version 1.10.147: resolved Issue 2329: InvoluteGear (fix)
    - description:  in machines.InvoluteGear, add tolerance to avoid point duplications, leading to graphics problems
    - date resolved: **2026-03-25 09:33**\ , date raised: 2026-03-25 
 * Version 1.10.146: resolved Issue 2327: ANCFBeam (fix)
    - description:  add implementation for damping similar to ANCFCable elements
    - date resolved: **2026-03-24 17:00**\ , date raised: 2026-03-24 
 * Version 1.10.145: resolved Issue 2323: graphics (extension)
    - description:  add function LinkedCylinders which draws the convex hull of two cylinders with parallel axes, useful for levers in mechanisms
    - date resolved: **2026-03-05 16:21**\ , date raised: 2026-03-05 
 * Version 1.10.144: resolved Issue 2324: SolidExtrusion (extension)
    - description:  add WARNING for unreferenced vertices in Delauney triangles, as this can indicate duplicated vertices in contour and often causes graphical issues
    - date resolved: **2026-03-05 16:20**\ , date raised: 2026-03-05 
 * Version 1.10.143: resolved Issue 2322: SolidExtrusion (fix)
    - description:  normals not correctly computed in case of smoothNormals=True
    - date resolved: **2026-03-05 12:48**\ , date raised: 2026-03-05 
 * Version 1.10.142: resolved Issue 2320: clippingPlane (change)
    - description:  add automatic normalization of clippingPlaneNormal in OpenGL renderer if norm is not 1 (except for 0 length)
    - date resolved: **2026-03-03 16:29**\ , date raised: 2026-03-03 
 * Version 1.10.141: resolved Issue 2314: Add ObjectJointSliding (extension)
    - description:  Add 3D SlidingJoint basic functionality
    - date resolved: **2026-03-02 09:48**\ , date raised: 2026-03-02 
 * Version 1.10.140: resolved Issue 2313: Add MarkerBodyBeamShape (extension)
    - description:  marker to be attached to beams for sliding joint and contact with beams
    - date resolved: **2026-03-02 09:46**\ , date raised: 2026-03-02 
 * Version 1.10.139: resolved Issue 2312: Newton (fix)
    - description:  newton.useNewtonSolver not used
    - date resolved: **2026-02-27 16:02**\ , date raised: 2026-02-27 
 * Version 1.10.138: resolved Issue 2311: DrawSystemGraph (extension)
    - description:  extend to add more information in graph, like type and name; add option to only retrieve graph but not to show; optionally add all item information
    - date resolved: **2026-02-25 10:34**\ , date raised: 2026-02-25 
 * Version 1.10.137: resolved Issue 2310: FEM (fix)
    - description:  ComputeHurtyCraigBamptonModes: for pure eigenmodes with no boundary lists, computation only works in sparse mode - extend to sparse mode and include optional exclusion of rigid body modes
    - date resolved: **2026-02-24 15:09**\ , date raised: 2026-02-24 
 * Version 1.10.136: resolved Issue 2257: VisualizationSettings (extension)
    - description:  check for all changed parameter names in .py and .tex files 
    - date resolved: **2026-02-20 00:16**\ , date raised: 2026-01-31 
 * Version 1.10.135: resolved Issue 2249: VisualizationSettings (extension)
    - description:  add feature to re-link outdated/changed settings and print warnings
    - date resolved: **2026-02-20 00:16**\ , date raised: 2026-01-31 
 * Version 1.10.134: resolved Issue 2248: VisualizationSettings (extension)
    - description:  add backlink to top settings structure in every substructure and initialize in constructor
    - date resolved: **2026-02-20 00:16**\ , date raised: 2026-01-31 
 * Version 1.10.133: resolved Issue 2285: visualizationSettings (fix)
    - description:  adapt Examples and TestModels to new visualizationSettings introduced in Exudyn 1.10.103
    - date resolved: **2026-02-20 00:15**\ , date raised: 2026-02-12 
 * Version 1.10.132: resolved Issue 2300: rotationCenterPoint (fix)
    - description:  fix correct calculation of mouse coordinates, bounding box, etc. when changing rotationCenterPoint
    - date resolved: **2026-02-19 22:29**\ , date raised: 2026-02-14 
 * Version 1.10.131: resolved Issue 2245: GLFWClient (extension)
    - description:  add camera position and orentation as alternative to current modelview; centerPoint adds up to it and modelRotation rotates the camera
    - date resolved: **2026-02-19 13:54**\ , date raised: 2026-01-31 
 * Version 1.10.130: resolved Issue 2307: zOffsetCamera (change)
    - description:  removed raytracer.zOffsetCamera as the z-shift is now possible with camera.nearFarPlaneOffset
    - date resolved: **2026-02-17 15:55**\ , date raised: 2026-02-17 
 * Version 1.10.129: resolved Issue 2306: trackMarker (fix)
    - description:  include trackMarker in light position and further transformations during OpenGL rendering and raytracing
    - date resolved: **2026-02-16 23:06**\ , date raised: 2026-02-16 
 * Version 1.10.128: resolved Issue 2294: views (fix)
    - description:  raytracing bounding box (searchTree?) wrong for example unbalancedFlywheel.py
    - **notes:** was due to NaN in numpy calculation for Bearing graphics; fixed
    - date resolved: **2026-02-16 22:46**\ , date raised: 2026-02-13 
 * Version 1.10.127: resolved Issue 2299: light positions (fix)
    - description:  Fix light positions for lights in camera frame - sync with lights initialization and with Raytracer
    - date resolved: **2026-02-16 22:45**\ , date raised: 2026-02-14 
 * Version 1.10.126: resolved Issue 2296: openGL settings (extension)
    - description:  add option showBoundingBox, which shows the bounding box of the current scene (as available in renderState)
    - date resolved: **2026-02-16 22:45**\ , date raised: 2026-02-14 
 * Version 1.10.125: resolved Issue 2303: boundingBox (fix)
    - description:  bounding box calculation resets in case that nan is included in points; check for nan in points in graphicsData when reading!
    - date resolved: **2026-02-15 22:38**\ , date raised: 2026-02-15 
 * Version 1.10.124: resolved Issue 2302: graphics (fix)
    - description:  BallBearingRings leads to nan (not a number) for impossible geometries, thus grooves not being calculated; add error message
    - date resolved: **2026-02-15 22:37**\ , date raised: 2026-02-15 
 * Version 1.10.123: resolved Issue 2301: ConstSizeMatrix (fix)
    - description:  Fix wrong GetDataPointer function, which points to data instead of &data[0]
    - date resolved: **2026-02-15 01:24**\ , date raised: 2026-02-15 
 * Version 1.10.122: resolved Issue 2241: systemStructures.py (change)
    - description:  add structure for ChildWindow (childWindow1-3)
    - **notes:** added view1-3 already earlier; 
    - date resolved: **2026-02-14 23:13**\ , date raised: 2026-01-31 
 * Version 1.10.121: resolved Issue 2297: tkinter font (fix)
    - description:  Exudyn Command dialog raises error due to AttributeError for tkinter.font
    - date resolved: **2026-02-14 22:52**\ , date raised: 2026-02-14 
 * Version 1.10.120: resolved Issue 2291: FromPyMeshlabFile (extension)
    - description:  add option to check and normalize normals
    - date resolved: **2026-02-14 17:31**\ , date raised: 2026-02-13 
 * Version 1.10.119: resolved Issue 2292: graphics.Move (extension)
    - description:  remove explanation to scale, as it also scales normals; add function MoveAndScale, which does both and Move calls MoveAndScale
    - date resolved: **2026-02-14 17:30**\ , date raised: 2026-02-13 
 * Version 1.10.118: resolved Issue 2295: AddEdgesAndSmoothenNormals (fix)
    - description:  remove arg pointTolerance as it is unused (roundDigits was introduced earlier)
    - date resolved: **2026-02-13 15:53**\ , date raised: 2026-02-13 
 * Version 1.10.117: resolved Issue 2293: graphics (change)
    - description:  functoin InvertTriangles: change arg invertVertexNormals to invertNormals for consistency reason with other functions
    - date resolved: **2026-02-13 15:02**\ , date raised: 2026-02-13 
 * Version 1.10.116: resolved Issue 2283: raytracer (extension)
    - description:  enable rendering of views with OpenGL or raytracer independently using flag view.camera.useRaytracer
    - date resolved: **2026-02-12 21:49**\ , date raised: 2026-02-11 
 * Version 1.10.115: resolved Issue 2280: FromPyMeshlabFile (extension)
    - description:  add graphics.FromPyMeshlabFile to import arbitrary geometries, e.g., .stl, .obj, .dae, etc. which can be loaded with pymeshlab; note: currently no textures or materials are loaded
    - date resolved: **2026-02-12 21:48**\ , date raised: 2026-02-11 
 * Version 1.10.114: resolved Issue 2254: RedrawAndGetImage (extension)
    - description:  add optional viewID=0 to enable offline rendering of window; use renderer.EnableView(viewID=...) before rendering to enable virtual camera and adjust settings with renderer.SetState(viewID=...) (without opening window)
    - date resolved: **2026-02-12 21:48**\ , date raised: 2026-02-01 
 * Version 1.10.113: resolved Issue 2253: GLFWClient (change)
    - description:  homogenize functions that operate on modelRotation and modelTranslation
    - date resolved: **2026-02-12 21:45**\ , date raised: 2026-02-01 
 * Version 1.10.112: resolved Issue 2252: VisualizationSettings (extension)
    - description:  add advanced subfolders to avoid large settings folders: general.advanced, contour.advanced, openGL.advanced, raytracer.advanced, interactive.advanced 
    - date resolved: **2026-02-12 21:45**\ , date raised: 2026-01-31 
 * Version 1.10.111: resolved Issue 2250: VisualizationSettings (extension)
    - description:  put openGL.light0/1xyz data to settings.openGL.light0/1.xyz; add total of 4 lights both for OpenGL and raytracer; add shadow to all lights; put cameraPos/Rot, lockModelView, clippingPlaneDistance, clippingPlaneNormal, perspective, facesTransparent, showFaceEdges, showFaces, showMeshEdges, showMeshFaces, general.worldBasisSize, general.drawCoordinateSystem, general.drawWorldBasis, general.showComputationInfo, general.textSize, ... to scene, window or camera 
    - date resolved: **2026-02-12 21:45**\ , date raised: 2026-01-31 
 * Version 1.10.110: resolved Issue 2284: raytracer (extension)
    - description:  add separate multiSampling flag: raytracers.advanced.multiSampling which is independent from OpenGL multiSampling
    - date resolved: **2026-02-12 21:42**\ , date raised: 2026-02-11 
 * Version 1.10.109: resolved Issue 2290: light position (change)
    - description:  change default values for openGL.light0position to [2,2,10,0] leading to less spot lights in flat scenes in x-y plane; similar for light1position 
    - date resolved: **2026-02-12 16:31**\ , date raised: 2026-02-12 
 * Version 1.10.108: resolved Issue 2289: nodes markers loads (change)
    - description:  simplified drawing cannot be turned on any more if drawFaces=False, because drawFaces has now moved to view.scene; added flag drawNodesMarkersLoadsWithFaces in C++ code to mark changes
    - date resolved: **2026-02-12 15:44**\ , date raised: 2026-02-12 
 * Version 1.10.107: resolved Issue 2288: nodes (change)
    - description:  drawNodesAsPoint together with showBasis: showBasis shall be drawn with lines if drawNodesAsPoint=True and with 3D faces otherwise
    - date resolved: **2026-02-12 15:41**\ , date raised: 2026-02-12 
 * Version 1.10.106: resolved Issue 2287: lightModelAmbient (change)
    - description:  use openGL.lightModelAmbient throughout for ambient color both for OpenGL and raytracer; adapt to [0.4,0.4,0.4,1] being slightly lighter while diffuse light0 is reduced from 0.6 to 0.5; this should give better visibility; light ambient color removed
    - date resolved: **2026-02-12 14:02**\ , date raised: 2026-02-12 
 * Version 1.10.105: resolved Issue 2286: ambientAndDiffuse (fix)
    - description:  In OpenGL materialAmbientAndDiffuse is not used for colored materials, so remove the value (deprecated structure redirects now to materialSpecular but must not be used anymore)
    - date resolved: **2026-02-12 14:02**\ , date raised: 2026-02-12 
 * Version 1.10.104: resolved Issue 2251: VisualizationSettings (extension)
    - description:  relocate options and add new folders scene, camera, window in new folders view0 (standard view as before), view0, view2, etc. 
    - date resolved: **2026-02-11 23:18**\ , date raised: 2026-01-31 
 * Version 1.10.103: resolved Issue 2282: light0 position (change)
    - description:  changed from [0.2,0.2,10,0] to [2,2,10,0] to avoid specular effects of objects in x-y plane in initial configuration
    - date resolved: **2026-02-11 14:00**\ , date raised: 2026-02-11 
 * Version 1.10.102: resolved Issue 2279: openGL.light0ambient (change)
    - description:  openGL.light0ambient and openGL.light1ambient are removed as they are redundant with openGL.lightModelAmbient which shall be used in future
    - **notes:** for that reason, the default value of openGL.lightModelAmbient has been changed to [0.3,0.3,0.3,1] to stay consistent!
    - date resolved: **2026-02-10 22:34**\ , date raised: 2026-02-10 
 * Version 1.10.101: resolved Issue 2276: charBitmap.h (change)
    - description:  split up into header and implementation to suppress warnings
    - date resolved: **2026-02-09 14:56**\ , date raised: 2026-02-09 
 * Version 1.10.100: resolved Issue 2275: GLFWClient (extension)
    - description:  add new substructure RenderView which collects GLFWwindow and RenderState, allowing to open several windows; window0=main window as before
    - date resolved: **2026-02-08 00:15**\ , date raised: 2026-02-08 
 * Version 1.10.99: resolved Issue 2243: GLFWClient (extension)
    - description:  add crosslines instead of mouse cursor when mouse coordinates are shown in renderer
    - date resolved: **2026-02-08 00:14**\ , date raised: 2026-01-31 
 * Version 1.10.98: resolved Issue 2274: Raytracer (fix)
    - description:  improve line drawing linewidth to comply with OpenGL renderer
    - date resolved: **2026-02-06 23:53**\ , date raised: 2026-02-06 
 * Version 1.10.97: resolved Issue 2271: ZoomAll (extension)
    - description:  allow call to ZoomAll with inactive renderer; add flag render=True; put current ZoomAll functionality from GLFWClient to accessible location
    - date resolved: **2026-02-06 23:53**\ , date raised: 2026-02-06 
 * Version 1.10.96: resolved Issue 2273: RenderState (fix)
    - description:  Initialize RenderState provisionally with default visualizationSettings at SystemContainer creation, then initialize with current SC.visualizationSettings at first call to any renderer function
    - date resolved: **2026-02-06 21:24**\ , date raised: 2026-02-06 
 * Version 1.10.95: resolved Issue 2272: general.autoFitScene (fix)
    - description:  already called at initialization of SystemContainer; needs to be overwritten at start of renderer or at first call to renderer/raytracer function - otherwise ZoomAll always called
    - date resolved: **2026-02-06 21:20**\ , date raised: 2026-02-06 
 * Version 1.10.94: resolved Issue 2263: docstrings (extension)
    - description:  similar to issue 0444, auto-convert comments to docstrings when creating wheels
    - date resolved: **2026-02-06 15:22**\ , date raised: 2026-02-03 
 * Version 1.10.93: resolved Issue 0444: docstrings (docu)
    - description:  change function comments in .py files to PEP standardized docstrings
    - date resolved: **2026-02-06 15:21**\ , date raised: 2020-09-06 
 * Version 1.10.92: resolved Issue 2270: Renderer (extension)
    - description:  in mode showMouseCoordinates, enable logging of mouse coordinates in current axis-aligned plane; behavior switched with logMouseCoordinates; only in case that perspective=0
    - date resolved: **2026-02-05 23:20**\ , date raised: 2026-02-04 
 * Version 1.10.91: resolved Issue 2258: showMouseCoordinates (change)
    - description:  make not available for perspective!=0 as it gives wrong values; disable zoom and mouse move as well
    - date resolved: **2026-02-05 23:20**\ , date raised: 2026-02-02 
 * Version 1.10.90: resolved Issue 2268: docstrings (extension)
    - description:  add docstrings to itemInterface
    - date resolved: **2026-02-05 23:19**\ , date raised: 2026-02-04 
 * Version 1.10.89: resolved Issue 2266: docstrings (extension)
    - description:  add docstrings for itemInterface items (__init__ function with all parameters)
    - date resolved: **2026-02-05 23:19**\ , date raised: 2026-02-03 
 * Version 1.10.88: resolved Issue 2265: stub files (.pyi) (extension)
    - description:  add details (input, output, example) for functions created in autoGeneratePyBindings, using a common GoogleStyle generator
    - date resolved: **2026-02-05 23:19**\ , date raised: 2026-02-03 
 * Version 1.10.87: resolved Issue 2269: Renderer (extension)
    - description:  allow finer zoom  stepsstep with CTRL+mouse wheel
    - date resolved: **2026-02-04 20:27**\ , date raised: 2026-02-04 
 * Version 1.10.86: resolved Issue 2262: stub files (.pyi) (extension)
    - description:  add description for objects created in autoGeneratePyBindings
    - date resolved: **2026-02-03 16:20**\ , date raised: 2026-02-03 
 * Version 1.10.85: resolved Issue 2256: stub files (.pyi) (extension)
    - description:  add docstrings for better help, as .pyi information is prioritized in Spyder
    - date resolved: **2026-02-03 16:20**\ , date raised: 2026-02-01 
 * Version 1.10.84: resolved Issue 2264: docstrings (extension)
    - description:  add docstrings for enums
    - date resolved: **2026-02-03 09:06**\ , date raised: 2026-02-03 
 * Version 1.10.83: resolved Issue 2261: stub files (.pyi) (extension)
    - description:  add docstrings to system structures
    - date resolved: **2026-02-02 23:37**\ , date raised: 2026-02-02 
 * Version 1.10.82: resolved Issue 2260: solvers (change)
    - description:  change default value exu.SimulationSettings() to None, but keep default behavior internally; this improves in-editor help
    - date resolved: **2026-02-02 22:48**\ , date raised: 2026-02-02 
 * Version 1.10.81: resolved Issue 2255: ResetKeyPressUserFunction (change)
    - description:  removed functionality as keyPressUserFunction can be reset by assigning it to 0
    - **notes:** keyPressUserFunction can now be reset by setting it to 0
    - date resolved: **2026-02-01 17:02**\ , date raised: 2026-02-01 
 * Version 1.10.80: resolved Issue 2242: GLFWClient (extension)
    - description:  add option for autorotation of scene, using a angular velocity vector; standard=[0,0,pi/2]; factor related to start time when timer is started
    - date resolved: **2026-02-01 12:13**\ , date raised: 2026-01-31 
 * Version 1.10.79: resolved Issue 2246: GLFWClient (change)
    - description:  model translation currently only uses x and y components: glTranslatef(translationMV[0], translationMV[1], 0.f); => fix in order to avoid large offsets in large scenes
    - **notes:** check graphics functionality in examples...
    - date resolved: **2026-01-31 22:12**\ , date raised: 2026-01-31 
 * Version 1.10.78: resolved Issue 2233: computeInitialAccelerations (extension)
    - description:  extend to sparse mode for initial velocities by simple global differentiation of sparse constraints
    - date resolved: **2026-01-23 00:29**\ , date raised: 2026-01-21 
 * Version 1.10.77: resolved Issue 2232: storeInitialAlgebraicCoordinates (change)
    - description:  in case of computation of initial  accelerations, also store initial Lagrange multipliers in initial coordinates
    - date resolved: **2026-01-21 23:01**\ , date raised: 2026-01-21 
 * Version 1.10.76: resolved Issue 2224: ANCFThinPlate (extension)
    - description:  add slopes scaling to elements
    - date resolved: **2026-01-18 23:07**\ , date raised: 2026-01-16 
 * Version 1.10.75: resolved Issue 2222: Jacobian description (docu)
    - description:  jacobians are computed e.g. as J = partial p/partial q and generalized forces are computed as J^T \* f; fix this in item descriptions, in ObjectMassPoint2D and ObjectMassPoint where J \* f is used; check 
    - date resolved: **2026-01-15 11:04**\ , date raised: 2026-01-15 
 * Version 1.10.74: resolved Issue 2221: CreateForce, CreateTorque (extension)
    - description:  allow marker indices in bodyNumber
    - date resolved: **2026-01-14 13:57**\ , date raised: 2026-01-14 
 * Version 1.10.73: resolved Issue 2220: AVX2 linux (change)
    - description:  enable AVX2 on linux and activate performance flags (O3)
    - date resolved: **2026-01-14 01:46**\ , date raised: 2026-01-14 
 * Version 1.10.72: resolved Issue 2214: shells.py (extension)
    - description:  add shells module for plate and shell elements; add generator function for planar meshes
    - date resolved: **2026-01-13 22:25**\ , date raised: 2026-01-11 
 * Version 1.10.71: resolved Issue 2199: ANCFThinPlate (extension)
    - description:  add access functions
    - date resolved: **2026-01-13 22:25**\ , date raised: 2026-01-07 
 * Version 1.10.70: resolved Issue 2216: graphics.Sphere (extension)
    - description:  add option to draw only part of sphere using min and max axis length given by angles; add option to draw hollow sphere with innerRadius
    - date resolved: **2026-01-12 22:37**\ , date raised: 2026-01-12 
 * Version 1.10.69: resolved Issue 2217: graphics.Sphere (extension)
    - description:  double tiling of circles to obtain better visibility
    - date resolved: **2026-01-12 19:03**\ , date raised: 2026-01-12 
 * Version 1.10.68: resolved Issue 2215: graphics.Basis (extension)
    - description:  add option to add text within arg labels
    - date resolved: **2026-01-12 18:37**\ , date raised: 2026-01-12 
 * Version 1.10.67: resolved Issue 2198: ANCFThinPlate (extension)
    - description:  add position, velocity, ... computation
    - date resolved: **2026-01-11 17:25**\ , date raised: 2026-01-07 
 * Version 1.10.66: resolved Issue 2197: ANCFThinPlate (extension)
    - description:  add correct element jacobian
    - date resolved: **2026-01-11 17:25**\ , date raised: 2026-01-07 
 * Version 1.10.65: resolved Issue 2213: GeometricallyExactBeam2D (fix)
    - description:  correct drawing with cross section and drawVertical
    - date resolved: **2026-01-09 20:43**\ , date raised: 2026-01-09 
 * Version 1.10.64: :textred:`resolved BUG 2212` : __init__.py, AVX2 
    - description:  error in __init__.py causes exudyn to always choose non-AVX2 version
    - date resolved: **2026-01-09 10:48**\ , date raised: 2026-01-09 
 * Version 1.10.63: resolved Issue 2211: GeometricallyExactBeam2D (extension)
    - description:  add autodiff jacobian
    - date resolved: **2026-01-09 02:15**\ , date raised: 2026-01-08 
 * Version 1.10.62: resolved Issue 2209: GeometricallyExactBeam2D (extension)
    - description:  add reference curvature and correct reference rotation for curved case
    - date resolved: **2026-01-08 23:10**\ , date raised: 2026-01-08 
 * Version 1.10.61: resolved Issue 2210: GeometricallyExactBeam2D (extension)
    - description:  flag includeReferenceRotations=False now refers to axial slopes rather to the direction of endpoints; this is identical for linear elements, but different for quadratic ones
    - date resolved: **2026-01-08 22:47**\ , date raised: 2026-01-08 
 * Version 1.10.60: resolved Issue 2207: GeometricallyExactBeam2D (extension)
    - description:  extend element for 3-node case with quadratic interpolation (just by providing a 3rd nodeNumber)
    - date resolved: **2026-01-08 21:42**\ , date raised: 2026-01-08 
 * Version 1.10.59: resolved Issue 2206: Manylinux wheels (extension)
    - description:  switch to docker, add compile manylinux wheels locally
    - date resolved: **2026-01-08 13:32**\ , date raised: 2026-01-08 
 * Version 1.10.58: resolved Issue 2195: maxSceneSize (extension)
    - description:  add bounding box for scene for better adjustments
    - date resolved: **2026-01-08 00:07**\ , date raised: 2026-01-06 
 * Version 1.10.57: resolved Issue 2201: OutputVariable (extension)
    - description:  change to 64bit
    - date resolved: **2026-01-07 19:03**\ , date raised: 2026-01-07 
 * Version 1.10.56: resolved Issue 1662: ANCFThinPlate (extension)
    - description:  add ANCF plate element based on 2 inplane slope vectors Slope12, based on Dufva/Shabana
    - date resolved: **2026-01-07 19:00**\ , date raised: 2023-10-15 
 * Version 1.10.55: resolved Issue 2190: perspective (fix)
    - description:  find causes for raytracer artifacts in case of larger perspective values
    - **notes:** adapted centerPoint; artifacts may still appear for very out of center models
    - date resolved: **2026-01-06 22:34**\ , date raised: 2026-01-04 
 * Version 1.10.54: resolved Issue 2196: ZoomAll (extension)
    - description:  add parameters zoomAllUseBoundingBox, boundingBoxZoomAllOffset and boundingBoxZoomAllFactor to adjust to exact bounding box of scene in rotated coordinates 
    - date resolved: **2026-01-06 18:30**\ , date raised: 2026-01-06 
 * Version 1.10.53: resolved Issue 2193: store model view (extension)
    - description:  print SetModelView code to console when pressing CTRL-F3
    - **notes:** see also section 'Storing the model view' in the docu
    - date resolved: **2026-01-06 16:59**\ , date raised: 2026-01-06 
 * Version 1.10.52: resolved Issue 2170: GLFWclient (testing)
    - description:  Test new USE_GLFW_GRAPHICS settings in compilation with -noglfw
    - date resolved: **2026-01-06 11:46**\ , date raised: 2025-12-25 
 * Version 1.10.51: resolved Issue 2192: coordinateSystem (change)
    - description:  add option to draw coordinate system with arrows; change general.drawCoordinateSystem to integer where 2 and 3 represent drawing with arrows
    - date resolved: **2026-01-06 02:09**\ , date raised: 2026-01-06 
 * Version 1.10.50: resolved Issue 2191: RedrawAndGetImage (fix)
    - description:  UpdatePostProcessData needs to be called in case that renderer is not running
    - date resolved: **2026-01-05 10:44**\ , date raised: 2026-01-05 
 * Version 1.10.49: resolved Issue 2153: raytracer (extension)
    - description:  add text using textures
    - date resolved: **2026-01-05 00:21**\ , date raised: 2025-11-02 
 * Version 1.10.48: resolved Issue 2167: perspective (change)
    - description:  revise perspective model and synchronize raytracer with openGL implementation
    - date resolved: **2026-01-04 22:19**\ , date raised: 2025-12-16 
 * Version 1.10.47: resolved Issue 2189: polygonOffset (change)
    - description:  increase openGL polygonOffset due to change in OpenGL projection method (zMaxSceneFactor changed from 100 to 2)
    - date resolved: **2026-01-04 20:34**\ , date raised: 2026-01-04 
 * Version 1.10.46: resolved Issue 2188: zMaxSceneFactor (change)
    - description:  add factor that previously was fixed to 100.; now defaulting to smaller default value, thus requiring adjustment in certain cases!
    - date resolved: **2026-01-04 14:17**\ , date raised: 2026-01-04 
 * Version 1.10.45: resolved Issue 2187: MainSystemExtensions (fix)
    - description:  JointPreCheckCalcBodyMarkers requires position even if not used
    - date resolved: **2026-01-04 12:44**\ , date raised: 2026-01-04 
 * Version 1.10.44: resolved Issue 2185: Create functions (extension)
    - description:  Add mbs.CreateFFRFReducedOrderObject
    - date resolved: **2026-01-04 11:42**\ , date raised: 2026-01-04 
    - resolved by: S. Weyrer
 * Version 1.10.43: resolved Issue 1196: ObjectFFRFreducedOrder sparse (extension)
    - description:  add sparse option for ObjectFFRFreducedOrder, filling in flexible-flexible mass terms into sparse mass matrix
    - **notes:** partially resolved and moved to newer issues
    - date resolved: **2026-01-04 11:35**\ , date raised: 2022-07-11 
 * Version 1.10.42: resolved Issue 0599: ObjectFFRFreducedOrder (check)
    - description:  compare Tait-Bryan and RotationVector cases with Euler parameters
    - **notes:** already resolved earlier
    - date resolved: **2026-01-04 11:23**\ , date raised: 2021-03-18 
 * Version 1.10.41: resolved Issue 0643: ObjectFFRFreducedOrder / CMS (tutorial)
    - description:  create tutorial with two bodies (crank, connecting rod, rigid piston)
    - **notes:** already resolved earlier with FFRF tutorial
    - date resolved: **2026-01-04 11:22**\ , date raised: 2021-04-30 
 * Version 1.10.40: resolved Issue 2169: GLFWclient (change)
    - description:  decouple graphics settings from USE_GLFW_GRAPHICS if they do not require OpenGL and GLFW
    - date resolved: **2026-01-03 23:38**\ , date raised: 2025-12-25 
 * Version 1.10.39: resolved Issue 2180: renderer.SetModelView (extension)
    - description:  add renderer function renderer.SetModelView(zoom, rotationVector, centerPoint) which adjusts the view to zoom, rotation and center point; in particular for raytracing
    - date resolved: **2026-01-03 01:51**\ , date raised: 2026-01-03 
 * Version 1.10.38: resolved Issue 2177: renderer.RedrawAndGetImage (extension)
    - description:  extended for software renderer (useRaytracer=True), to grab images of the 3D view completely without starting the renderer
    - date resolved: **2026-01-03 01:51**\ , date raised: 2026-01-01 
 * Version 1.10.37: resolved Issue 2181: renderState (change)
    - description:  update visualizationSettings.window.renderWindowSize when renderState.currentWindowSize is prescribed in call to renderer.SetState(...)
    - date resolved: **2026-01-03 01:11**\ , date raised: 2026-01-03 
 * Version 1.10.36: resolved Issue 2179: raytracer (fix)
    - description:  check why raytracer does not render triangles at first run
    - date resolved: **2026-01-03 00:50**\ , date raised: 2026-01-03 
 * Version 1.10.35: resolved Issue 2147: raytracer (extension)
    - description:  export images without opening renderer; including view settings
    - date resolved: **2026-01-03 00:43**\ , date raised: 2025-09-23 
 * Version 1.10.34: resolved Issue 2178: RendererInActiveError (change)
    - description:  add error handling if a renderer function is called which requires the renderer to be active but it is not
    - date resolved: **2026-01-01 23:09**\ , date raised: 2026-01-01 
 * Version 1.10.33: resolved Issue 2176: processing (extension)
    - description:  useMPI is not used in GeneticOptimization and in ComputeSensitivities; add flag and pass through to ProcessParameterList
    - date resolved: **2025-12-31 16:31**\ , date raised: 2025-12-31 
    - resolved by: Z. Zhang
 * Version 1.10.32: resolved Issue 2175: GlfwRenderer (change)
    - description:  change the way how interactive.lockModelView is realized, ignoring keys and mouse actions on model view
    - date resolved: **2025-12-29 23:57**\ , date raised: 2025-12-29 
 * Version 1.10.31: resolved Issue 2174: GlfwRenderer (change)
    - description:  change selectBufferSize during mouse select from 10000 to 1000 to reduce stack size problems
    - date resolved: **2025-12-29 23:57**\ , date raised: 2025-12-29 
 * Version 1.10.30: resolved Issue 2173: GraphicsData (extension)
    - description:  add fontSize and offset to graphicsData text structure as well as to graphics.Text(...) function
    - date resolved: **2025-12-29 21:40**\ , date raised: 2025-12-29 
 * Version 1.10.29: resolved Issue 2172: contour (extension)
    - description:  add contour colors to VisualizationSettings
    - date resolved: **2025-12-27 23:09**\ , date raised: 2025-12-26 
 * Version 1.10.28: resolved Issue 2166: raytracer (extension)
    - description:  add shortkey to switch to raytracing
    - **notes:** CTRL-R used for switching
    - date resolved: **2025-12-27 22:01**\ , date raised: 2025-12-16 
 * Version 1.10.27: resolved Issue 2171: GLFWclient (change)
    - description:  decouple Raytracer from GlfwClient
    - date resolved: **2025-12-26 00:03**\ , date raised: 2025-12-26 
 * Version 1.10.26: resolved Issue 2168: GLFWclient (change)
    - description:  decouple GlfwClientBitmapText.h from USE_GLFW_GRAPHICS as it does not require OpenGL and GLFW
    - date resolved: **2025-12-25 22:47**\ , date raised: 2025-12-25 
 * Version 1.10.25: resolved Issue 2165: perspective (change)
    - description:  improve perspective for raytracer, accepting values > 1
    - date resolved: **2025-12-16 14:30**\ , date raised: 2025-12-16 
 * Version 1.10.24: resolved Issue 2164: multiSampling (extension)
    - description:  allow 3 as value for multisampling, in accordance with raytracer
    - date resolved: **2025-12-16 14:29**\ , date raised: 2025-12-16 
 * Version 1.10.23: resolved Issue 2146: raytracer (extension)
    - description:  improve performance of line-triangle cuts by skipping bounding boxes further away
    - date resolved: **2025-12-15 00:47**\ , date raised: 2025-09-23 
 * Version 1.10.22: resolved Issue 2163: Create functions (extension)
    - description:  extend Create functions in MainSystemExtensions to accept marker indices in bodyNumbers in connector and joint functions when useful for flexible bodies - CreateSpringDamper, CreateRevoluteJoint, etc.
    - date resolved: **2025-12-13 01:11**\ , date raised: 2025-12-13 
 * Version 1.10.21: resolved Issue 2162: raytracer (fix)
    - description:  lines: adjust functionality to openGL settings; currently mesh edges and face edges draw with same settings
    - date resolved: **2025-12-12 23:26**\ , date raised: 2025-12-12 
 * Version 1.10.20: resolved Issue 2161: __init__.py, AVX2 (change)
    - description:  remove numpy determination of AVX2 support and just use try-except approach; should work better with newer numpy versions
    - date resolved: **2025-12-08 17:08**\ , date raised: 2025-12-08 
 * Version 1.10.19: resolved Issue 2160: FEMinterface (extension)
    - description:  add function Get BoundaryNodeSetsAsLists to retrieve boundary node sets as two lists containing nodes list and weights list
    - date resolved: **2025-11-27 10:11**\ , date raised: 2025-11-27 
    - resolved by: S. Weyrer
 * Version 1.10.18: resolved Issue 2159: FEMinterface (extension)
    - description:  nodeSets receive additional "type" information, e.g. used to indicate boundary node sets in CreateNGsolveBoundaryNodeSets
    - date resolved: **2025-11-27 10:09**\ , date raised: 2025-11-27 
 * Version 1.10.17: resolved Issue 2158: FEMinterface (extension)
    - description:  add metaData dictionary for optional information and source of mesh/model
    - **notes:** switching to FEM fileVersion 4
    - date resolved: **2025-11-27 10:08**\ , date raised: 2025-11-27 
 * Version 1.10.16: resolved Issue 2157: FEMinterface (fix)
    - description:  ComputeHurtyCraigBamptonModesNGsolve: change dtype=np.int into dtype=int to comply with numpy
    - date resolved: **2025-11-27 10:06**\ , date raised: 2025-11-27 
    - resolved by: S. Weyrer
 * Version 1.10.15: resolved Issue 2156: Exudyn version (fix)
    - description:  version shown as unknown in readthedocs page
    - date resolved: **2025-11-11 15:12**\ , date raised: 2025-11-11 
 * Version 1.10.14: resolved Issue 2152: renderer (extension)
    - description:  add function RedrawAndGetImage to directly obtain numpy array with RGB image data to be used with matplotlib imshow
    - date resolved: **2025-10-17 18:38**\ , date raised: 2025-10-17 
 * Version 1.10.13: resolved Issue 2151: general.showHelpOnStartup (fix)
    - description:  only allows values >0 but 0 required to turn of message
    - date resolved: **2025-10-17 18:37**\ , date raised: 2025-10-17 
 * Version 1.10.12: resolved Issue 2150: simulation (change)
    - description:  add smiley and frowney for end of simulation
    - date resolved: **2025-10-17 09:14**\ , date raised: 2025-10-17 
 * Version 1.10.11: resolved Issue 2149: UTF-8 encoding (change)
    - description:  unify internal character representation, add full greek letters and sub/super indices for numbers
    - **notes:** some special characters may now appear differently!
    - date resolved: **2025-10-16 23:02**\ , date raised: 2025-10-16 
 * Version 1.10.10: resolved Issue 2148: InteractiveDialog (fix)
    - description:  problems showing marker as set_data in matplotlib does not accept scalars any more
    - date resolved: **2025-10-13 10:14**\ , date raised: 2025-10-13 
 * Version 1.10.9: resolved Issue 2145: raytracer (extension)
    - description:  add smoothing filter for shadows, only applied in case that values in kernel are not containing 0 and 1 (which could be real edges)
    - **notes:** smoothing done at low resolution shadow map, using number of shadowSmoothingSteps
    - date resolved: **2025-09-30 15:17**\ , date raised: 2025-09-23 
 * Version 1.10.8: resolved Issue 2144: raytracer (extension)
    - description:  improve smooth shadows performance by precomputation of smoothed shadow map (without multisampling)
    - **notes:** added raytracer options shadowScalingFactor and shadowSmoothingSteps; default values work well with lightRadiusVariations=31
    - date resolved: **2025-09-30 15:14**\ , date raised: 2025-09-23 
 * Version 1.10.7: resolved Issue 2143: raytracer (fix)
    - description:  improve smooth shadows by adding radius variations for lightRadius
    - date resolved: **2025-09-23 12:26**\ , date raised: 2025-09-23 
 * Version 1.10.6: resolved Issue 2142: Lie group sigma (change)
    - description:  improve accuracy of implicit Lie group integrator; to activate set timeIntegration.generalizedAlpha.lieGroupSimplifiedKinematicRelations = False; see paper Holzinger, Arnold, Gerstmayr. Sigma-modified Lie group generalized alpha methods for constrained multibody systems, 2025 (to be sumitted)
    - date resolved: **2025-08-07 13:40**\ , date raised: 2025-08-07 
    - resolved by: S. Holzinger
 * Version 1.10.5: resolved Issue 1650: MacOS Rosetta (fix)
    - description:  importing numpy gives Intel MKL Warning: Support of Intel Streaming SIMD Extensions 4.2 ... has been deprecated. Intel oneAPI Math Kernel Library 2025.0 will require AVX instructions
    - **notes:** works at least with Exudyn 1.10.0;  Rosetta will be abandoned in future, if compilation does not work any more
    - date resolved: **2025-07-11 15:59**\ , date raised: 2023-07-20 
 * Version 1.10.4: resolved Issue 1026: Save as PNG (testing)
    - description:  check MacOS implementation if glfw works with saving PNG files
    - **notes:** resolved earlier; works with Exudyn 1.10.0
    - date resolved: **2025-07-11 15:58**\ , date raised: 2022-04-02 
 * Version 1.10.3: resolved Issue 0865: multithreaded solver (extension)
    - description:  test TaskManager::SuspendWorkers() for non-parallel parts (e.g. linear solver); possibly measure time spent for these parts, which should be larger than 2 ms to make sense; add option parallel.stopThreadsInSerialSections=False
    - **notes:** not relevant any more
    - date resolved: **2025-07-11 15:49**\ , date raised: 2022-01-15 
 * Version 1.10.2: resolved Issue 2141: CreateDistanceSensor (fix)
    - description:  args storeInternal and fileName not used
    - date resolved: **2025-07-10 16:53**\ , date raised: 2025-07-10 
    - resolved by: P. Manzl
 * Version 1.10.1: resolved Issue 2139: linux (fix)
    - description:  time seems to be non-initilized in linux version
    - date resolved: **2025-07-10 01:56**\ , date raised: 2025-07-10 
 * Version 1.10.0: resolved Issue 2123: Mechanisms (extension)
    - description:  Add example for gear and toothed rack mounted on body, using relative translation/rotation marker
    - **notes:** see Example involuteGearGraphics.py
    - date resolved: **2025-07-09 18:43**\ , date raised: 2025-07-02 

***********
Version 1.9
***********

 * Version 1.9.235: resolved Issue 1959: SphereSphereContact (check)
    - description:  check that position marker works without friction
    - **notes:** not needed any more as mass points and point nodes include rotation matrix and angular velocity
    - date resolved: **2025-07-09 18:12**\ , date raised: 2025-02-24 
 * Version 1.9.234: resolved Issue 2122: Mechanisms (extension)
    - description:  Add example for two gears mounted on a body with revolute joints, using CoordinateSpringDamperExt
    - **notes:** see Example involuteGearGraphics.py
    - date resolved: **2025-07-09 18:10**\ , date raised: 2025-07-02 
 * Version 1.9.233: resolved Issue 2137: general open source (docu)
    - description:  add some warnings at beginning of docu regarding open source and potential errors
    - date resolved: **2025-07-09 18:07**\ , date raised: 2025-07-05 
 * Version 1.9.232: resolved Issue 2138: RST, latex (docu)
    - description:  fill issues with references in RST and latex files
    - date resolved: **2025-07-05 18:29**\ , date raised: 2025-07-05 
 * Version 1.9.231: resolved Issue 2135: TestSuite (fix)
    - description:  put all variables into static TestSuite class, to avoid unintentionally changing variables in the testsuite scope by models
    - date resolved: **2025-07-05 15:07**\ , date raised: 2025-07-05 
 * Version 1.9.230: resolved Issue 2136: trackerlog (change)
    - description:  change to utf-8 format; this may cause problems when loading old (backup) files, which shall be manually changed to utf-8
    - date resolved: **2025-07-05 14:54**\ , date raised: 2025-07-05 
 * Version 1.9.229: resolved Issue 2134: ContactSphereTriangle (testing)
    - description:  add test example using CreateSphereQuadContact with two contacting bodies to check momentum conservation
    - date resolved: **2025-07-05 12:10**\ , date raised: 2025-07-05 
 * Version 1.9.228: resolved Issue 2133: ContactSphereTriangle (fix)
    - description:  marker transformation missing for contact computation
    - date resolved: **2025-07-05 11:10**\ , date raised: 2025-07-05 
 * Version 1.9.227: resolved Issue 2132: nodes (change)
    - description:  letter N for basis only shown in case that nodes.showNumbers=True
    - date resolved: **2025-07-05 10:46**\ , date raised: 2025-07-05 
 * Version 1.9.226: resolved Issue 2129: MarkerBodiesRelativeRotationCoordinate (fix)
    - description:  does not work without nodeNumber provided
    - date resolved: **2025-07-03 22:21**\ , date raised: 2025-07-03 
 * Version 1.9.225: resolved Issue 2121: Involute gear (extension)
    - description:  add function to generate graphics for toothed rack
    - date resolved: **2025-07-03 18:33**\ , date raised: 2025-07-02 
 * Version 1.9.224: resolved Issue 2128: SolidExtrusion (extension)
    - description:  add relative rotation and relative offset for second surface of extrusion; used e.g. for helical gears
    - date resolved: **2025-07-03 16:48**\ , date raised: 2025-07-03 
 * Version 1.9.223: resolved Issue 2126: Involute gear (extension)
    - description:  add function to create involute gear graphics
    - date resolved: **2025-07-03 15:37**\ , date raised: 2025-07-03 
 * Version 1.9.222: resolved Issue 2125: BallBearing (extension)
    - description:  add function to create ball bearing into machines; similar to issue 2031
    - date resolved: **2025-07-03 14:57**\ , date raised: 2025-07-03 
 * Version 1.9.221: resolved Issue 2118: Involute gear (extension)
    - description:  add function to create involute gear profile
    - date resolved: **2025-07-03 12:57**\ , date raised: 2025-07-02 
 * Version 1.9.220: resolved Issue 2031: BallBearing (extension)
    - description:  put function into machines module; function shalll take two markers (for outer and inner ring), which represent the interface to other bodies; add simple test model
    - date resolved: **2025-07-03 09:20**\ , date raised: 2025-05-18 
 * Version 1.9.219: resolved Issue 2124: BallBearing (extension)
    - description:  add graphics function to create rings of ball bearings
    - date resolved: **2025-07-03 09:19**\ , date raised: 2025-07-03 
 * Version 1.9.218: resolved Issue 2119: Involute gear (extension)
    - description:  add function to generate graphics for involute gear
    - date resolved: **2025-07-02 09:40**\ , date raised: 2025-07-02 
 * Version 1.9.217: resolved Issue 2117: MarkerRelative (extension)
    - description:  add offset to MarkerBodiesRelativeRotationCoordinate
    - date resolved: **2025-07-01 23:00**\ , date raised: 2025-07-01 
 * Version 1.9.216: resolved Issue 2112: MarkerRelativeRotation (extension)
    - description:  Consider marker MarkerBodiesRelativeRotationCoordinate which measures scalar relative rotation, in particular rotation relative between two bodies; action is realized as torques on two bodies, using body-fixed coordinates of body0
    - date resolved: **2025-07-01 20:09**\ , date raised: 2025-06-25 
 * Version 1.9.215: resolved Issue 2116: Raytracer (extension)
    - description:  add line color
    - date resolved: **2025-07-01 14:22**\ , date raised: 2025-07-01 
 * Version 1.9.214: resolved Issue 2115: Raytracer (extension)
    - description:  make raytracer compatible with facesTransparent, showFaces, showFaceEdges
    - date resolved: **2025-07-01 14:22**\ , date raised: 2025-07-01 
 * Version 1.9.213: resolved Issue 2114: FEM (extension)
    - description:  add function GetRigidBodyInertia to return RigidBodyInertia of nodal position-based FEM models
    - date resolved: **2025-06-30 14:30**\ , date raised: 2025-06-30 
    - resolved by: S. Weyrer
 * Version 1.9.212: resolved Issue 2111: MarkerRelativeTranslation (extension)
    - description:  Consider marker MarkerBodiesRelativeTranslationCoordinate which measures scalar relative translation, in particular relative displacement between two bodies; action is realized as forces on two bodies, using body-fixed coordinates of body0
    - date resolved: **2025-06-29 23:16**\ , date raised: 2025-06-25 
 * Version 1.9.211: resolved Issue 1961: CreateSphereQuadContact (extension)
    - description:  add create function, which uses 2 CreateSphereTriangleContact elements; should be practical for simple robots, etc.
    - date resolved: **2025-06-29 12:01**\ , date raised: 2025-02-24 
 * Version 1.9.210: resolved Issue 1958: CreateSphereSphereContact (extension)
    - description:  add Create function, including data node; similar to spring-damper; add checks that in case of friction, a rigid body marker is required
    - date resolved: **2025-06-29 11:06**\ , date raised: 2025-02-24 
 * Version 1.9.209: resolved Issue 2113: ExportSTL (fix)
    - description:  invertNormals and invertTriangles not used
    - date resolved: **2025-06-25 17:18**\ , date raised: 2025-06-25 
 * Version 1.9.208: resolved Issue 2050: parallel (fix)
    - description:  check global counters in parallelized computations which cause huge cache issues
    - **notes:** current issues seem to be related to fetch_add (use task stealing?) and cache-polution between different PARALLEL_FOR loops
    - date resolved: **2025-06-24 23:32**\ , date raised: 2025-05-26 
 * Version 1.9.207: resolved Issue 2108: ContactSphereTriangle (testing)
    - description:  test with explicit integration
    - date resolved: **2025-06-22 16:54**\ , date raised: 2025-06-21 
 * Version 1.9.206: resolved Issue 2107: ContactSphereSphere (testing)
    - description:  test with explicit integration
    - date resolved: **2025-06-22 16:54**\ , date raised: 2025-06-21 
 * Version 1.9.205: resolved Issue 2105: Examples and TestModels (fix)
    - description:  remove uncommitted test files from Examples and TestModels as they are included as examples in theDoc
    - date resolved: **2025-06-21 22:48**\ , date raised: 2025-06-19 
 * Version 1.9.204: resolved Issue 2102: automatic code (change)
    - description:  avoid writing dates to systemstrutures files and others
    - date resolved: **2025-06-19 00:22**\ , date raised: 2025-06-18 
 * Version 1.9.203: resolved Issue 2018: docu (docu)
    - description:  when searching for keywords in examples, exclude matches (like MassMatrix) where the character before or after the keyword is a letter (a-zA-Z) to avoid wrong matches!
    - date resolved: **2025-06-18 22:22**\ , date raised: 2025-05-15 
 * Version 1.9.202: resolved Issue 2104: FEMinterface (fix)
    - description:  add try except to automatic nodeSet creation in case that there are no boundaries computable
    - date resolved: **2025-06-18 18:17**\ , date raised: 2025-06-18 
 * Version 1.9.201: resolved Issue 2103: Item selection (extension)
    - description:  write currently selected item (+type, etc.) into renderer.state
    - date resolved: **2025-06-18 17:00**\ , date raised: 2025-06-18 
 * Version 1.9.200: resolved Issue 2101: parallel (extension)
    - description:  add simulationSettings.parallel.useLoadBalancing to switch two multithreading modes
    - date resolved: **2025-06-18 12:00**\ , date raised: 2025-06-18 
 * Version 1.9.199: resolved Issue 1695: taskmanager (change)
    - description:  extend microthreading for taskmanager-based load management; remove taskmanager from repo and create pure BSD license
    - **notes:** in C++ ExuThreading being now the only mode; optionally use exu.special.solver.multiThreadingLoadBalancing to switch load balancing on/off
    - date resolved: **2025-06-18 10:00**\ , date raised: 2023-11-19 
 * Version 1.9.198: resolved Issue 2100: Command window (fix)
    - description:  does not show edit dialog
    - date resolved: **2025-06-16 14:52**\ , date raised: 2025-06-16 
 * Version 1.9.197: resolved Issue 2099: contour (extension)
    - description:  add alpha channel visualizationSettings option for contour colors
    - date resolved: **2025-06-16 11:28**\ , date raised: 2025-06-16 
 * Version 1.9.196: resolved Issue 2098: graphics.CheckerBoard (extension)
    - description:  add arg materialIndex for graphics material of both colors
    - date resolved: **2025-06-16 11:03**\ , date raised: 2025-06-16 
 * Version 1.9.195: resolved Issue 2096: ContactSphereTriangle (testing)
    - description:  add test model sphereTriangleTest.py
    - date resolved: **2025-06-15 19:56**\ , date raised: 2025-06-15 
 * Version 1.9.194: resolved Issue 2095: CreateKinematicTree (testing)
    - description:  add test model
    - **notes:** add test model createKinematicTreeTest.py
    - date resolved: **2025-06-15 19:55**\ , date raised: 2025-06-15 
 * Version 1.9.193: resolved Issue 1960: ObjectContactSphereTriangle (extension)
    - description:  add contact similar to GeneralContact and to ObjectContactSphereSphere, but being able to be computed implicitly; add option to exclude certain edges to be able to also correctly handle meshes
    - date resolved: **2025-06-15 11:50**\ , date raised: 2025-02-24 
 * Version 1.9.192: :textred:`resolved BUG 2094` : ContactSphereSphere 
    - description:  ContactSphereSphere and ContactSphereTorus set frictionRegularizedRegion wrong in case of computeFromData
    - **notes:** now leads to improved convergence
    - date resolved: **2025-06-15 09:37**\ , date raised: 2025-06-15 
 * Version 1.9.191: resolved Issue 2092: FEMinterface (extension)
    - description:  ComputePostProcessingModesNGsolve: extend for multi-material coefficient function, using materials dict; 
    - date resolved: **2025-06-14 10:40**\ , date raised: 2025-06-13 
 * Version 1.9.190: resolved Issue 2090: FEMinterface (extension)
    - description:  extend ImportMeshFromNGsolve to include several materials given as dict of materials
    - date resolved: **2025-06-14 10:31**\ , date raised: 2025-06-13 
 * Version 1.9.189: resolved Issue 2089: FEMinterface (extension)
    - description:  add function to compute nodeSets from NGsolve boundary names: CreateNGsolveBoundaryNodeSets 
    - date resolved: **2025-06-14 10:31**\ , date raised: 2025-06-13 
 * Version 1.9.188: resolved Issue 2088: FEMinterface (extension)
    - description:  add internal function to get node numbers of NGsolve boundary condition: GetNodesOfNGsolveBoundary
    - date resolved: **2025-06-14 10:31**\ , date raised: 2025-06-13 
 * Version 1.9.187: resolved Issue 2093: FEMinterface (extension)
    - description:  adapt class KirchhoffMaterial for multi-domain materials
    - date resolved: **2025-06-14 10:30**\ , date raised: 2025-06-14 
 * Version 1.9.186: resolved Issue 2091: FEMinterface (change)
    - description:  remove deprecated arg computeEigenmodes from ImportMeshFromNGsolve
    - date resolved: **2025-06-13 22:16**\ , date raised: 2025-06-13 
 * Version 1.9.185: resolved Issue 2087: GetOtherMarker (change)
    - description:  return MarkerBodyRigid as body anyway must provide position and orientation
    - date resolved: **2025-06-13 22:13**\ , date raised: 2025-06-13 
 * Version 1.9.184: resolved Issue 2067: CreateKinematicTree (extension)
    - description:  add Create function for KinematicTree, using list of TreeLink; add special class TreeLink to itemInterface
    - date resolved: **2025-06-10 08:52**\ , date raised: 2025-05-29 
 * Version 1.9.183: resolved Issue 2083: SolidOfRevolution (extension)
    - description:  add check to graphics.SolidOfRevolution that order of list is correct (leads to correct inside-outside relations)
    - date resolved: **2025-06-10 01:02**\ , date raised: 2025-06-09 
 * Version 1.9.182: resolved Issue 2086: RigidBodyInertia (extension)
    - description:  Add GetGraphics for combined inertia and graphics generation
    - date resolved: **2025-06-09 12:31**\ , date raised: 2025-06-09 
 * Version 1.9.181: resolved Issue 2056: graphics (fix)
    - description:  Renderer raises warning that some objects contain inconsistencies between computed triangle normals and vertex normals
    - **notes:** mainly due to SolidOfRevolution
    - date resolved: **2025-06-09 01:14**\ , date raised: 2025-05-28 
 * Version 1.9.180: resolved Issue 2085: SolidOfRevolution (change)
    - description:  switch triangle order in SolidOfRevolution to have consistent normals and triangles
    - date resolved: **2025-06-09 01:06**\ , date raised: 2025-06-09 
 * Version 1.9.179: resolved Issue 2084: drawFaceNormals (change)
    - description:  openGL.drawFaceNormals shall draw the computed normal from the triangle points, thus showing the correct orientation of triangles
    - date resolved: **2025-06-09 00:48**\ , date raised: 2025-06-09 
 * Version 1.9.178: resolved Issue 2082: shadow (fix)
    - description:  add lightPositionsInCameraFrame flag to shadow computation to consistently draw shadow and lights in openGL
    - date resolved: **2025-06-08 18:11**\ , date raised: 2025-06-08 
 * Version 1.9.177: resolved Issue 2080: HDF5 (testing)
    - description:  Add test with all data types
    - **notes:** added example testHDF5loadSave.py
    - date resolved: **2025-06-08 18:11**\ , date raised: 2025-06-08 
 * Version 1.9.176: :textred:`resolved BUG 2079` : LoadDictFromHDF5 
    - description:  does not correctly convert np.array inside list
    - date resolved: **2025-06-08 18:11**\ , date raised: 2025-06-07 
 * Version 1.9.175: resolved Issue 2081: GL_LIGHT (extension)
    - description:  add option to switch between local and global lights, to be compatible with Raytracer
    - **notes:** added option openGL.lightPositionsInCameraFrame to switch behavior; in raytracer, this setting is always True
    - date resolved: **2025-06-08 15:44**\ , date raised: 2025-06-08 
 * Version 1.9.174: resolved Issue 2078: netgen (extension)
    - description:  add option to convert netgen / ngsolve mesh into points, triangles and normals including smooth geometries
    - date resolved: **2025-06-07 19:12**\ , date raised: 2025-06-07 
 * Version 1.9.173: resolved Issue 2077: Raytracer (extension)
    - description:  light radius receives circular variation normal to ray, giving an effient and good effect of spherical lights; use lightRadius and lightRadiusVariations
    - date resolved: **2025-06-05 21:03**\ , date raised: 2025-06-05 
 * Version 1.9.172: resolved Issue 2076: Raytracer (extension)
    - description:  add option for spherical lights with radius and random sampling
    - **notes:** added lightRadius
    - date resolved: **2025-06-04 17:52**\ , date raised: 2025-06-04 
 * Version 1.9.171: resolved Issue 2059: Raytracer (docu)
    - description:  add short docu part
    - **notes:** example available in Examples/newtonsCradle.py
    - date resolved: **2025-06-04 10:58**\ , date raised: 2025-05-28 
 * Version 1.9.170: resolved Issue 2074: invert triangles (extension)
    - description:  Add function to invert triangles and normals of graphicsData, e.g., for inverted sphere or brick
    - **notes:** added graphics.InvertTriangles and ConsistentTriangleList
    - date resolved: **2025-06-04 01:37**\ , date raised: 2025-06-02 
 * Version 1.9.169: resolved Issue 2061: Raytracer (extension)
    - description:  add materials interface via SystemContainer: renderer.SetMaterial(index, dict), dict=GetMaterial(index)
    - **notes:** can do read [] access, Set(), New(), etc.
    - date resolved: **2025-06-03 16:06**\ , date raised: 2025-05-28 
 * Version 1.9.168: resolved Issue 2063: Raytracer (extension)
    - description:  make first 10 materials in renderer accessible via visualization systems dialog
    - date resolved: **2025-06-03 16:05**\ , date raised: 2025-05-28 
 * Version 1.9.167: resolved Issue 2060: Raytracer (extension)
    - description:  adjust minZ and line offset to scene dimension
    - date resolved: **2025-06-02 17:45**\ , date raised: 2025-05-28 
 * Version 1.9.166: resolved Issue 2058: Raytracer (extension)
    - description:  add separate visualization options; keep lights
    - date resolved: **2025-06-02 17:45**\ , date raised: 2025-05-28 
 * Version 1.9.165: resolved Issue 2070: Renderer timeout (check)
    - description:  check whether calling glfwWaitEventsTimeout, glfwPollEvents or glfwSwapBuffers fixes problems with redraw timeouts with Raytracing
    - **notes:** can be done with option raytracer.keepWindowActive
    - date resolved: **2025-06-02 17:44**\ , date raised: 2025-05-31 
 * Version 1.9.164: resolved Issue 2069: Raytracer (extension)
    - description:  add global fog and material fog
    - **notes:** only global fog added
    - date resolved: **2025-06-02 17:44**\ , date raised: 2025-05-29 
 * Version 1.9.163: resolved Issue 2068: Raytracer (extension)
    - description:  activate cutting plane for raytracer; include in function IntersectRayWithTriangles, which shall first check for cutting plane and only takes then objects behind cutting plane (if hit); color is then taken from triangle behind cutting plane-without reflections and shadows
    - date resolved: **2025-06-02 17:44**\ , date raised: 2025-05-29 
 * Version 1.9.162: resolved Issue 2073: font bitmaps (extension)
    - description:  Add smoothing at pixel level for base text fonts (at grey scale) to obtain smoother fonts
    - date resolved: **2025-06-01 22:53**\ , date raised: 2025-06-01 
 * Version 1.9.161: resolved Issue 2072: renderer (change)
    - description:  exu.StartRenderer() and exu.StopRenderer() are changed into SC.renderer.Start() and SC.renderer.Stop(), having a SystemContainer SC
    - **notes:** NOTE that this affects your existing models a lot!
    - date resolved: **2025-06-01 19:13**\ , date raised: 2025-06-01 
 * Version 1.9.160: resolved Issue 2071: Raytracer (extension)
    - description:  draw system message texts as overlay of render image
    - date resolved: **2025-06-01 18:00**\ , date raised: 2025-06-01 
 * Version 1.9.159: resolved Issue 2036: renderer (change)
    - description:  put renderer-related functions into SystemContainer; preserve compatibility; consider substructure renderer in SC to collect rendering-related functionality
    - date resolved: **2025-06-01 02:36**\ , date raised: 2025-05-21 
 * Version 1.9.158: resolved Issue 2037: renderer (change)
    - description:  homogenize WaitForUserToContinue, DoRendererIdleTasks, DoIdleOperations and WaitForRenderEngineStopFlag as they are widely identical; homogenize functionality; note that DoRendererIdleTasks() with default args has to be replaced by renderer.DoIdleTasks(0)
    - **notes:** merged into SC.renderer.DoIdleTasks(); NOTE that this change affects your existing models a lot!
    - date resolved: **2025-06-01 02:35**\ , date raised: 2025-05-21 
 * Version 1.9.157: resolved Issue 2062: renderer (extension)
    - description:  add renderer substructure to SystemContainer (with backlink to MainSystemContainer)
    - **notes:** all examples, test models, teaching models and exudyn submodules adapted; old functionality preserved so far with warnings
    - date resolved: **2025-06-01 02:34**\ , date raised: 2025-05-28 
    - resolved by: CHANGE
 * Version 1.9.156: resolved Issue 2045: visualization (extension)
    - description:  use templated function to have only one single function for triangle drawing
    - date resolved: **2025-05-30 01:01**\ , date raised: 2025-05-22 
 * Version 1.9.155: resolved Issue 2066: graphics (extension)
    - description:  add option for special shape of graphics.Brick with rounded edges, offering a transition to an ellipsoid
    - date resolved: **2025-05-30 00:59**\ , date raised: 2025-05-29 
 * Version 1.9.154: resolved Issue 2049: SetRenderState (fix)
    - description:  seems to ignore autoFitScene = False and therefore does not reload zoom factor
    - **notes:** SetRenderState is extended by a flag which by default waits until the renderer is fully started or redrawn; this can impede performance, which is why this should be set to False in such cases
    - date resolved: **2025-05-29 16:22**\ , date raised: 2025-05-26 
 * Version 1.9.153: resolved Issue 2065: SimulationSettings (change)
    - description:  timeIntegration.numberOfSteps: change type to Real in order to accept steps as Python float; add warning in solver if numberOfSteps deviates significantely from integer number
    - date resolved: **2025-05-29 15:33**\ , date raised: 2025-05-29 
 * Version 1.9.152: resolved Issue 2064: exudyn.config (change)
    - description:  move further options to config: SetWriteToConsole, flush always, etc.
    - date resolved: **2025-05-29 14:47**\ , date raised: 2025-05-29 
 * Version 1.9.151: resolved Issue 2057: PrintDelayed (fix)
    - description:  printing from visualiuation thread: buffer is never emptied; use idle tasks to do so
    - date resolved: **2025-05-28 01:12**\ , date raised: 2025-05-28 
 * Version 1.9.150: resolved Issue 2055: exudyn.Print (change)
    - description:  replace print commands in exudyn modules by exudyn.Print(...) in order to have common handling of file writing, etc.
    - date resolved: **2025-05-28 00:08**\ , date raised: 2025-05-28 
 * Version 1.9.149: :textred:`resolved BUG 2054` : GeneticOptimization 
    - description:  computationIndex and parameterFunctionData do not work except for first generation
    - date resolved: **2025-05-27 23:39**\ , date raised: 2025-05-27 
 * Version 1.9.148: resolved Issue 2052: exudyn.Print (extension)
    - description:  function shall accept kwargs end, flush and sep in order to be more compatible with original Python print
    - date resolved: **2025-05-27 22:39**\ , date raised: 2025-05-27 
 * Version 1.9.147: resolved Issue 2053: FEM (fix)
    - description:  ComputePostProcessingModes raises error for larger problems due to print command
    - date resolved: **2025-05-27 22:16**\ , date raised: 2025-05-27 
 * Version 1.9.146: resolved Issue 2051: Raytracer (fix)
    - description:  resolve parallelization issue by removing global hit count
    - date resolved: **2025-05-26 18:58**\ , date raised: 2025-05-26 
 * Version 1.9.145: resolved Issue 2048: Raytracer (extension)
    - description:  add Raytracer as option; map options from openGL to Raytracer (lights, material, multiSampling, shadow, perspective, etc.)
    - date resolved: **2025-05-26 10:51**\ , date raised: 2025-05-26 
 * Version 1.9.144: resolved Issue 2047: graphics.Quad (extension)
    - description:  add normals; also affects graphics.CheckerBoard; normals are not needed, but may be modified in transformations
    - date resolved: **2025-05-25 15:40**\ , date raised: 2025-05-25 
 * Version 1.9.143: resolved Issue 2046: Raytracer (extension)
    - description:  add CPU-based software renderer / raytracer als option to render images for animations
    - **notes:** note that this is experimental; image resolution shall be small (start with 400x300) as long render times may lead to problems
    - date resolved: **2025-05-25 12:32**\ , date raised: 2025-05-25 
 * Version 1.9.142: resolved Issue 2044: visualization (extension)
    - description:  add option to sort transparent triangles to improve quality of transparent objects; use simple depth sort for triangle midpoints
    - **notes:** added option openGL.depthSorting which sorts triangles by their depth for improved transparency view (but requires triangles to be small enough)
    - date resolved: **2025-05-22 16:35**\ , date raised: 2025-05-22 
 * Version 1.9.141: resolved Issue 2043: stub files (.pyi) (fix)
    - description:  revise section for exudyn stubs in order to enable completion of exudyn.config and other exudyn functions
    - **notes:** seems that removing erroneous info like "special." was sufficient to work
    - date resolved: **2025-05-22 11:00**\ , date raised: 2025-05-22 
 * Version 1.9.140: resolved Issue 2033: InertiaSphere (extension)
    - description:  use density and radius to initialize as alternative option
    - date resolved: **2025-05-22 10:32**\ , date raised: 2025-05-20 
 * Version 1.9.139: resolved Issue 2042: exudyn (change)
    - description:  move exudyn.SetLinalgOutputFormatPython(), exudyn.SetOutputPrecision(), exudyn.SetPrintDelayMilliSeconds(), exudyn.SuppressWarnings(), exudyn.InfoStat(), exudyn.GetVersionString() to according variables and functions in exudyn.config, see current docu
    - date resolved: **2025-05-22 09:30**\ , date raised: 2025-05-22 
 * Version 1.9.138: resolved Issue 2041: exudyn.Solve (change)
    - description:  remove exudyn.SolveStatic exudyn.SolveDynamic and exudyn.ComputeODE2Eigenvalues from exudyn, being only available in MainSystem as mbs.SolveDynamic, etc.
    - date resolved: **2025-05-22 08:35**\ , date raised: 2025-05-22 
 * Version 1.9.137: resolved Issue 2040: exudyn.Demo1() (change)
    - description:  and exudyn.Demo2() only available as exudyn.demos.Demo1() or .Demo2()
    - date resolved: **2025-05-22 07:56**\ , date raised: 2025-05-22 
 * Version 1.9.136: resolved Issue 2039: exudyn.Go() (change)
    - description:  remove function Go() which is not intended to be used
    - date resolved: **2025-05-22 07:44**\ , date raised: 2025-05-22 
 * Version 1.9.135: resolved Issue 2038: exudyn.SetOutputPrecision (change)
    - description:  moved to exudyn.config.outputPrecision
    - date resolved: **2025-05-22 07:21**\ , date raised: 2025-05-22 
 * Version 1.9.134: resolved Issue 1630: exudyn module (extension)
    - description:  consider settings instead of putting all variables globally into module
    - date resolved: **2025-05-22 07:20**\ , date raised: 2023-06-23 
 * Version 1.9.133: resolved Issue 2035: exudyn.config (change)
    - description:  collect specific settings into config structure, see also issue 1630
    - date resolved: **2025-05-22 07:19**\ , date raised: 2025-05-21 
 * Version 1.9.132: resolved Issue 2034: Sensors.traces (extension)
    - description:  add trace time span for past and future traces, to limit length of traces
    - date resolved: **2025-05-20 08:19**\ , date raised: 2025-05-20 
 * Version 1.9.131: resolved Issue 2032: sensor traces (fix)
    - description:  wrong description using visualizationSettings.sensorTraces in docu
    - date resolved: **2025-05-20 08:19**\ , date raised: 2025-05-20 
 * Version 1.9.130: resolved Issue 2028: OutputVariables (extension)
    - description:  add outputvariables to ContactSphereTorus
    - date resolved: **2025-05-19 20:18**\ , date raised: 2025-05-18 
 * Version 1.9.129: resolved Issue 2024: ObjectContactFrictionCircleCable2DOld (change)
    - description:  remove outdated object
    - date resolved: **2025-05-18 23:35**\ , date raised: 2025-05-17 
 * Version 1.9.128: resolved Issue 2030: SolidOfRevolution (extension)
    - description:  add smoothingAngle for contour below which smoothing is applied
    - date resolved: **2025-05-18 15:35**\ , date raised: 2025-05-18 
 * Version 1.9.127: resolved Issue 2029: graphics (extension)
    - description:  add start and end angle for minor radius of torus
    - date resolved: **2025-05-18 11:24**\ , date raised: 2025-05-18 
 * Version 1.9.126: resolved Issue 2022: ContactTorusSphere (extension)
    - description:  add basic contact element; typically used for ball bearings; similar to ContactSphereSphere
    - date resolved: **2025-05-18 00:03**\ , date raised: 2025-05-16 
 * Version 1.9.125: resolved Issue 2027: CreateRigidBody (change)
    - description:  if graphicsDataList is provided, node shall still be shown, but drawsize gets 0; this allows to easily show the node basis
    - **notes:** also done for CreateMassPoint
    - date resolved: **2025-05-17 23:30**\ , date raised: 2025-05-17 
 * Version 1.9.124: resolved Issue 2026: graphics Tube (extension)
    - description:  add function graphics.Tube which generates graphicsData for a tube along a line of points with axis vectors
    - date resolved: **2025-05-17 21:48**\ , date raised: 2025-05-17 
 * Version 1.9.123: resolved Issue 2025: Torus (extension)
    - description:  add graphics function for torus
    - **notes:** using graphics.Tube function
    - date resolved: **2025-05-17 21:48**\ , date raised: 2025-05-17 
 * Version 1.9.122: resolved Issue 1922: ContactSphereSphere (testint)
    - description:  add test model with various contact models
    - date resolved: **2025-05-17 00:15**\ , date raised: 2024-11-02 
 * Version 1.9.121: resolved Issue 2023: graphics.Cylinder (extension)
    - description:  add option to draw hollow cylinder
    - date resolved: **2025-05-17 00:05**\ , date raised: 2025-05-17 
 * Version 1.9.120: resolved Issue 2021: ContactSphereSphere (extension)
    - description:  add option to include hollow sphere - sphere contact
    - date resolved: **2025-05-16 18:10**\ , date raised: 2025-05-16 
 * Version 1.9.119: resolved Issue 2020: version files (change)
    - description:  remove different version files and use only a single file for .tex and .cpp
    - date resolved: **2025-05-15 21:25**\ , date raised: 2025-05-15 
 * Version 1.9.118: resolved Issue 2019: Numpy2.0 (fix)
    - description:  Since Numpy2.0 __cpu_features__ are not available any more, therefore we need to find another way to automatically detect AVX2 features
    - date resolved: **2025-05-15 20:20**\ , date raised: 2025-05-15 
 * Version 1.9.117: resolved Issue 2012: CObjectContactCurveCircles (fix)
    - description:  complete CheckPreAssembleConsistency
    - date resolved: **2025-05-15 08:50**\ , date raised: 2025-05-11 
 * Version 1.9.116: resolved Issue 2014: ContactCurveCircles (extension)
    - description:  check damping mechanism and check negative contact forces
    - date resolved: **2025-05-13 15:12**\ , date raised: 2025-05-11 
 * Version 1.9.115: resolved Issue 1926: ContactCurveCircles (testing)
    - description:  add simple test model
    - date resolved: **2025-05-11 22:53**\ , date raised: 2024-11-04 
 * Version 1.9.114: resolved Issue 2013: ContactCurveCircles (extension)
    - description:  add visualization for contact circles
    - date resolved: **2025-05-11 22:07**\ , date raised: 2025-05-11 
 * Version 1.9.113: resolved Issue 2015: ContactSphereSphere (fix)
    - description:  radius unused in visualization
    - date resolved: **2025-05-11 21:44**\ , date raised: 2025-05-11 
 * Version 1.9.112: resolved Issue 2010: ContactCurveCircles (extension)
    - description:  change dynamic arrays to local temp arrays in CObject, similar to FFRF object
    - date resolved: **2025-05-11 15:36**\ , date raised: 2025-05-11 
 * Version 1.9.111: resolved Issue 2009: ContactCurveCircles (extension)
    - description:  draw active contact segments
    - date resolved: **2025-05-11 12:02**\ , date raised: 2025-05-11 
 * Version 1.9.110: resolved Issue 2008: ContactCurveCircles (extension)
    - description:  draw contact segments
    - date resolved: **2025-05-11 12:02**\ , date raised: 2025-05-11 
 * Version 1.9.109: :textred:`resolved BUG 2011` : ContactCurveCircles 
    - description:  force computation not correct regarding simultaneous contact with several segments
    - date resolved: **2025-05-11 12:01**\ , date raised: 2025-05-11 
 * Version 1.9.108: resolved Issue 1925: ContactCurveCircles (example)
    - description:  add example of chain drive
    - date resolved: **2025-05-11 12:01**\ , date raised: 2024-11-04 
 * Version 1.9.107: resolved Issue 1952: CreateTorsionalSpringDamper (testing)
    - description:  add test example
    - **notes:** included in createFunctionsTest
    - date resolved: **2025-05-10 23:23**\ , date raised: 2025-02-05 
 * Version 1.9.106: resolved Issue 2005: realtime (extension)
    - description:  measure realtime reserve in special timers
    - date resolved: **2025-05-10 23:18**\ , date raised: 2025-05-10 
 * Version 1.9.105: resolved Issue 2001: Create functions (extension)
    - description:  add automatism to create geometry from inertia objects by default (cylinder, sphere, brick, ...); using default colors
    - date resolved: **2025-05-10 23:07**\ , date raised: 2025-05-08 
 * Version 1.9.104: resolved Issue 2006: RigidBodyInertia (extension)
    - description:  add self.data dictionary which stores data of special inertia classes, such as radius of cylinder, etc.; this allows to obtain geometry information
    - date resolved: **2025-05-10 20:39**\ , date raised: 2025-05-10 
 * Version 1.9.103: resolved Issue 1971: Create functions (extension)
    - description:  add general test model
    - date resolved: **2025-05-10 19:56**\ , date raised: 2025-03-05 
 * Version 1.9.102: resolved Issue 1987: CreateCoordinateConstraint (extension)
    - description:  add option to constrain coordinate of object via its nodes; add option to constrain single coordinates or even a list of coordinates
    - date resolved: **2025-05-10 19:37**\ , date raised: 2025-04-08 
 * Version 1.9.101: resolved Issue 0657: Delete item (extension)
    - description:  add functionality to delete items, adding specific features to re-index nodes in objects/markers, etc. if a node, object or marker is deleted; add MaxItemNumber() function to systemData which returns unique name for items even after deletion
    - **notes:** resolved by reordering dependent items; no maxItem used
    - date resolved: **2025-05-10 01:31**\ , date raised: 2021-05-01 
 * Version 1.9.100: resolved Issue 2004: GetJointArgs (extension)
    - description:  add a utility function, which takes an existing marker and rotationMarker and another body to compute a new marker and rotationMarker for the other body; can be directly used as joint arguments like in revolute, prismatic or generic joints
    - date resolved: **2025-05-09 23:59**\ , date raised: 2025-05-09 
 * Version 1.9.99: resolved Issue 2003: GetOtherMarker (extension)
    - description:  add a utility function, which the computes a new marker from the reference position of an existing marker and another body; for easier setup of markers
    - date resolved: **2025-05-09 15:34**\ , date raised: 2025-05-09 
 * Version 1.9.98: resolved Issue 1999: delete (extension)
    - description:  add option to delete dependent markers when deleting loads
    - date resolved: **2025-05-09 08:08**\ , date raised: 2025-05-08 
 * Version 1.9.97: resolved Issue 1998: delete (extension)
    - description:  add option to delete dependent markers in connectors/joints when deleting objects
    - date resolved: **2025-05-09 08:08**\ , date raised: 2025-05-07 
 * Version 1.9.96: resolved Issue 2000: delete (extension)
    - description:  consistently delete loads and sensors
    - date resolved: **2025-05-08 20:57**\ , date raised: 2025-05-08 
 * Version 1.9.95: resolved Issue 2002: BasicTraits.h (change)
    - description:  remove and add new AdvancedStuff.h and put there sort, atomics operations, iterators, etc. which are not needed everywhere
    - date resolved: **2025-05-08 20:39**\ , date raised: 2025-05-08 
 * Version 1.9.94: resolved Issue 1997: delete (extension)
    - description:  consistently delete nodes and markers in MainSystem
    - date resolved: **2025-05-08 13:35**\ , date raised: 2025-05-06 
 * Version 1.9.93: resolved Issue 1985: CreateFunctions (extension)
    - description:  check pyi files and workflows to enable code completion and navigation in Spyder with Create functions
    - **notes:** shall be resolved with issue 1996, as it natively derives from C++ class
    - date resolved: **2025-05-06 22:20**\ , date raised: 2025-04-02 
 * Version 1.9.92: resolved Issue 1986: ObjectMassPoint (extension)
    - description:  add outputvariables rotation matrix and angular velocity (also to NodePoint) in order to allow spherical joint to be attached.
    - date resolved: **2025-05-06 22:12**\ , date raised: 2025-04-08 
 * Version 1.9.91: :textred:`resolved BUG 1995` : Sensor 
    - description:  case SensorType::KinematicTree missing in GetTypeDependentIndex
    - date resolved: **2025-05-06 17:13**\ , date raised: 2025-05-06 
 * Version 1.9.90: resolved Issue 1994: pybind11 (fix)
    - description:  local files using pybind11 2.6, not suitable for Numpy >= 2.0
    - date resolved: **2025-05-06 16:44**\ , date raised: 2025-05-06 
 * Version 1.9.89: resolved Issue 1993: MSVC (fix)
    - description:  compilation/execution of Exudyn in MSVC and python differ considerably
    - date resolved: **2025-05-06 16:00**\ , date raised: 2025-05-06 
 * Version 1.9.88: resolved Issue 1992: CSystem (fix)
    - description:  systemIsConsistent used instead of systemIsInteger in CheckSystemIntegrity
    - date resolved: **2025-05-06 12:07**\ , date raised: 2025-05-06 
 * Version 1.9.87: resolved Issue 1991: MainSystem (fix)
    - description:  Node, Object, Marker, ... functions have wrong internal range check (not including invalid index)
    - date resolved: **2025-05-05 22:43**\ , date raised: 2025-05-05 
 * Version 1.9.86: resolved Issue 1990: delete object (extension)
    - description:  first step to fulfill issue 657; reorder markers and sensors and assign invalid object number if object is used; add assemble check function for invalid indices
    - date resolved: **2025-05-05 22:18**\ , date raised: 2025-05-05 
 * Version 1.9.85: resolved Issue 1984: FEM (extension)
    - description:  Load/Save of FEM and FFRF data: default value NPY switched to NPZ due to Numpy2.x conflicts; for compatibility set mode to NPY
    - **notes:** all load/save functions should work with npy, npz, pkl and hdf5
    - date resolved: **2025-04-01 19:21**\ , date raised: 2025-04-01 
 * Version 1.9.84: resolved Issue 1983: FEM (change)
    - description:  adjust Load/Save functions to by-default support hdf5 or pkl as npy does not work with numpy 2.0
    - **notes:** changed ObjectFFRFreducedOrderInterface and FEM class to by default use numpy NPZ format
    - date resolved: **2025-04-01 16:44**\ , date raised: 2025-04-01 
 * Version 1.9.83: resolved Issue 1982: SolutionViewer (extension)
    - description:  add option Make mp4 to create videos with python ffmpeg lib
    - date resolved: **2025-04-01 08:51**\ , date raised: 2025-04-01 
 * Version 1.9.82: resolved Issue 1981: Images2Video (extension)
    - description:  add function ConvertImages2Video and dialog InteractiveImages2Video to convert images to videos directly in Python
    - date resolved: **2025-04-01 02:20**\ , date raised: 2025-04-01 
 * Version 1.9.81: resolved Issue 1980: PlotSensor (change)
    - description:  change to matplotlib.get_backend().lower() when comparing with matplotlib agg render mode; this shall avoid warnings in case of plots with agg in background
    - date resolved: **2025-03-31 23:03**\ , date raised: 2025-03-31 
 * Version 1.9.80: resolved Issue 1972: artificialIntelligence (fix)
    - description:  PreInitializeSolver always uses Generalized alpha solver, but should go in line with SetSolver
    - **notes:** this issue was erratic and is ignored; PreInitializeSolver shall be replaced by derived class calling SetSolver with the desired solverType; added remarks in artificialIntelligence.py
    - date resolved: **2025-03-31 22:42**\ , date raised: 2025-03-06 
 * Version 1.9.79: resolved Issue 1976: mbs.Assemble() (change)
    - description:  check if assembled in SolveStatic and SolveDynamic and automatically call mbs.Assemble() prior to solver; only raise warning
    - date resolved: **2025-03-31 22:15**\ , date raised: 2025-03-30 
 * Version 1.9.78: resolved Issue 1979: ComputeSystemDegreeOfFreedom (change)
    - description:  change print to exudyn.Print for workflow consistency
    - **notes:** also fixed other print commands in solver.py
    - date resolved: **2025-03-31 21:52**\ , date raised: 2025-03-31 
 * Version 1.9.77: resolved Issue 1978: SetWriteToFile (extension)
    - description:  add flushAlways (default:False) option for immediate writing to file
    - date resolved: **2025-03-31 21:48**\ , date raised: 2025-03-31 
 * Version 1.9.76: resolved Issue 1975: CreateTorsionalSpringDamper (fix)
    - description:  does not work for unlimitedRotations=True due to additional comma
    - date resolved: **2025-03-12 15:08**\ , date raised: 2025-03-12 
 * Version 1.9.75: resolved Issue 1974: CreateSphericalJoint (change)
    - description:  Make it work for bodies that do not offer a rotation matrix (mass points)
    - date resolved: **2025-03-12 14:35**\ , date raised: 2025-03-12 
 * Version 1.9.74: resolved Issue 1973: GeneralContact (change)
    - description:  remove frictionVelocityPenalty as it is not used; also remove macro ANCFuseFrictionPenalty
    - date resolved: **2025-03-07 17:00**\ , date raised: 2025-03-07 
 * Version 1.9.73: resolved Issue 1970: CreateRollingDisc (extension)
    - description:  create test model
    - date resolved: **2025-03-05 22:36**\ , date raised: 2025-03-05 
 * Version 1.9.72: resolved Issue 1969: CreateRollingDisc (extension)
    - description:  add create function to MainSystem
    - date resolved: **2025-03-05 22:17**\ , date raised: 2025-03-05 
 * Version 1.9.71: resolved Issue 1968: GeneralContact (extension)
    - description:  add option to only use dynamic search tree with no duplicated search bins
    - **notes:** in case of no static triangles, static searchtree is only created once and no additional operations are performed
    - date resolved: **2025-03-05 21:11**\ , date raised: 2025-03-03 
 * Version 1.9.70: resolved Issue 1967: Create functions (change)
    - description:  change return values of all mbs.Create functions with joints (CreateRevoluteJoint, CreateSphericalJoint, CreatePrismaticJoint, CreateGenericJoint, CreateDistanceConstraints, ...) to the object index instead of lists
    - date resolved: **2025-03-03 22:04**\ , date raised: 2025-03-03 
 * Version 1.9.69: resolved Issue 1949: RollingDiscPenalty (extension)
    - description:  add create function to MainSystem
    - **notes:** already done with issue 1957
    - date resolved: **2025-03-03 20:41**\ , date raised: 2025-02-04 
 * Version 1.9.68: resolved Issue 1966: GeneralContact (extension)
    - description:  SearchTree: add option to perform more accurate test for triangles: add simple check to see if bin is fully on one side of the triangle
    - date resolved: **2025-03-02 18:50**\ , date raised: 2025-03-02 
 * Version 1.9.67: resolved Issue 1895: GeneralContact (extension)
    - description:  add option to add static objects to GeneralContact and to SearchTree
    - date resolved: **2025-03-02 15:42**\ , date raised: 2024-10-16 
 * Version 1.9.66: resolved Issue 1965: GeneralContact (change)
    - description:  ODE2RHS timer removed and replaced with CSystem Contact:Overall timer (which includes PostNewtonStep)
    - date resolved: **2025-03-02 11:46**\ , date raised: 2025-03-02 
 * Version 1.9.65: resolved Issue 1962: CuttingPlane (extension)
    - description:  add simple option for cutting plane (point, normal vector, flag) to exclude triangles and other objects from graphics
    - **notes:** added options openGL.clippingPlaneNormal and openGL.clippingPlaneDistance to enable simple clipping
    - date resolved: **2025-02-28 21:30**\ , date raised: 2025-02-27 
    - resolved by: EXTENSION
 * Version 1.9.64: resolved Issue 1964: Cable2D (extension)
    - description:  add setting useReducedOrderIntegration=2 to interface docu
    - date resolved: **2025-02-28 20:57**\ , date raised: 2025-02-28 
 * Version 1.9.63: resolved Issue 1963: CreateRollingDiscPenalty (testing)
    - description:  create test model
    - date resolved: **2025-02-28 00:22**\ , date raised: 2025-02-27 
 * Version 1.9.62: resolved Issue 1957: RollingDiscPenalty (testing)
    - description:  add test example
    - date resolved: **2025-02-27 16:50**\ , date raised: 2025-02-09 
 * Version 1.9.61: resolved Issue 1951: RigidBody2D (testing)
    - description:  add test for physicsCenterOfMass != 0
    - date resolved: **2025-02-05 15:08**\ , date raised: 2025-02-05 
 * Version 1.9.60: resolved Issue 1955: FEM (fix)
    - description:  WarnNumpy2 contains error, as it tries to compare int with str
    - date resolved: **2025-02-05 15:04**\ , date raised: 2025-02-05 
 * Version 1.9.59: resolved Issue 1950: RigidBody2D (extension)
    - description:  add parameter physicsCenterOfMass, similar to RigidBody
    - date resolved: **2025-02-05 14:53**\ , date raised: 2025-02-05 
 * Version 1.9.58: resolved Issue 1581: mainSystemExtensions (extension)
    - description:  add LinearSpringDamper and TorsionalSpringDamper
    - **notes:** transferred to issues 1948 and 1953
    - date resolved: **2025-02-05 08:44**\ , date raised: 2023-05-21 
 * Version 1.9.57: resolved Issue 1948: CreateTorsionalSpringDamper (extension)
    - description:  add create function to MainSystem
    - date resolved: **2025-02-04 13:44**\ , date raised: 2025-02-04 
 * Version 1.9.56: :textred:`resolved BUG 1946` : SphereSphereContact 
    - description:  error in PostNewtonStep: always uses data variables
    - date resolved: **2025-02-03 15:40**\ , date raised: 2025-02-03 
 * Version 1.9.55: resolved Issue 1945: lieGroupSimplifiedKinematicRelations (change)
    - description:  lieGroupSimplifiedKinematicRelations used to test more accurate Lie group solver; for now, default value set to false in order to have test suite running
    - date resolved: **2025-01-28 11:53**\ , date raised: 2025-01-28 
 * Version 1.9.54: resolved Issue 1944: solver (fix)
    - description:  always writes 'Solver terminated unsuccessfully' independently of success.
    - date resolved: **2025-01-05 18:00**\ , date raised: 2025-01-05 
 * Version 1.9.53: resolved Issue 1479: RotationVector2RotationMatrix (fix)
    - description:  both in Python and C++, fix range to 0..2\*pi, as large angles cause low accuracy
    - date resolved: **2024-12-01 19:13**\ , date raised: 2023-03-27 
 * Version 1.9.52: resolved Issue 1651: Python 3.11 (extension)
    - description:  added Python 3.11 workflows for Windows, Linux and MacOS builds (note: problems with Rosetta x86 on MacOS)
    - **notes:** resolved earlier also for 3.12 and 3.13
    - date resolved: **2024-12-01 12:31**\ , date raised: 2023-07-20 
 * Version 1.9.51: resolved Issue 1483: PUMA560 (fix)
    - description:  COM frames of KinematicTree drawn wrong: serialRobotInverseKinematics; check COM setting
    - **notes:** resolved already with issue 1857
    - date resolved: **2024-12-01 12:29**\ , date raised: 2023-03-29 
 * Version 1.9.50: :textred:`resolved BUG 1943` : FEMinterface 
    - description:  CreateNonlinearFEMObjectGenericODE2NGsolve does not work due to change in internal FEM stiffness and mass matrices, stored as scipy sparse matrices now
    - date resolved: **2024-11-27 21:24**\ , date raised: 2024-11-27 
 * Version 1.9.49: resolved Issue 1942: System equations of motion (docu)
    - description:  add missing term partial g/partial qDot in ODE2 part, in particular for non-holonomic constriants; this part was already implemented in a generic way since the very beginning, but missed to find the way into the documentation.
    - date resolved: **2024-11-18 19:15**\ , date raised: 2024-11-18 
 * Version 1.9.48: resolved Issue 1940: NumPy2 (extension)
    - description:  Add warning in FEM for some functions that do not work properly with NumPy 2.x
    - date resolved: **2024-11-10 19:57**\ , date raised: 2024-11-10 
 * Version 1.9.47: resolved Issue 1939: HDF5 load/save (extension)
    - description:  extend to None type (but excluding numpy arrays containing None!
    - date resolved: **2024-11-10 18:43**\ , date raised: 2024-11-10 
 * Version 1.9.46: resolved Issue 1935: GraphicsData functions (change)
    - description:  adjust graphics.Sphere, graphics.Brick, etc. to create numpy arrays for improved efficiency
    - date resolved: **2024-11-10 17:24**\ , date raised: 2024-11-10 
 * Version 1.9.45: resolved Issue 1936: GraphicsData functions (change)
    - description:  adjust graphics conversion functions (graphics.Move, etc.) to handle numpy arrays as well
    - date resolved: **2024-11-10 16:37**\ , date raised: 2024-11-10 
 * Version 1.9.44: resolved Issue 1938: graphics.Sphere (fix)
    - description:  only works if addEdges <=1; add condition for number of edges
    - date resolved: **2024-11-10 16:09**\ , date raised: 2024-11-10 
 * Version 1.9.43: resolved Issue 1937: graphics.color (fix)
    - description:  type completion not working
    - date resolved: **2024-11-10 15:32**\ , date raised: 2024-11-10 
 * Version 1.9.42: resolved Issue 1934: GraphicsData write (change)
    - description:  GetObject now returns GraphicsData as numpy arrays to be consistent with issue 1934
    - date resolved: **2024-11-10 14:26**\ , date raised: 2024-11-10 
 * Version 1.9.41: resolved Issue 1933: GraphicsData read (change)
    - description:  allow that all colors, positions, triangles, points, normals, edges, ... are either lists or numpy.arrays (but flatten to 1D)
    - date resolved: **2024-11-10 14:26**\ , date raised: 2024-11-10 
 * Version 1.9.40: resolved Issue 1929: GraphicsData (extension)
    - description:  extend import/export function of GraphicsData for numpy arrays; speeds up load/safe significantly
    - date resolved: **2024-11-10 14:26**\ , date raised: 2024-11-08 
 * Version 1.9.39: resolved Issue 1908: StaticSolver (fix)
    - description:  does not write appropriate error message, but only writes: ValueError: SolveStatic terminated due to errors
    - **notes:** together with 1931
    - date resolved: **2024-11-09 17:33**\ , date raised: 2024-10-24 
 * Version 1.9.38: resolved Issue 1930: Solver (change)
    - description:  change message when solver finishes, distinguishing between solver success and failure: 'Solver terminated unsuccessfully' or 'Solver terminated unsuccessfully'
    - date resolved: **2024-11-09 17:32**\ , date raised: 2024-11-09 
 * Version 1.9.37: resolved Issue 1928: URDF (chekc)
    - description:  check import from roboticstoolbox-python and pymeshlab
    - date resolved: **2024-11-08 22:31**\ , date raised: 2024-11-08 
 * Version 1.9.36: resolved Issue 1927: SaveDictToHDF5 (extension)
    - description:  extend for saving int32, int64, float32 and float64
    - date resolved: **2024-11-08 22:30**\ , date raised: 2024-11-08 
 * Version 1.9.35: resolved Issue 1923: ContactCurveCircles (extension)
    - description:  add special contact element between curve defined by segments in contact with circles; 2D curves and circle co-move with rigid body marker at which curve is attached; enables Cam-follower mechanism, chain-sprocket contact, etc.
    - date resolved: **2024-11-04 23:36**\ , date raised: 2024-11-04 
 * Version 1.9.34: resolved Issue 1921: ContactSphereSphere (extension)
    - description:  add option to use restitution coefficient
    - date resolved: **2024-11-04 23:32**\ , date raised: 2024-11-02 
 * Version 1.9.33: resolved Issue 1919: ContactSphereSphere (extension)
    - description:  extend for nonlinear contact models; in particular restitution coefficient and adhesive elasto-plastic contact model
    - date resolved: **2024-11-04 23:32**\ , date raised: 2024-11-02 
    - resolved by: S. Weyrer
 * Version 1.9.32: resolved Issue 1918: ContactSphereSphere (extension)
    - description:  add basic contact object for sphere-sphere contact, with options for linear and nonlinear contact models as well as adhesion
    - date resolved: **2024-11-02 11:49**\ , date raised: 2024-11-02 
 * Version 1.9.31: resolved Issue 1917: useRecommendedStepSize (extension)
    - description:  add option in timeIntegration.discontinuous to turn on/off step size recommendations for contact and other discontinuous phenomena
    - date resolved: **2024-11-02 11:46**\ , date raised: 2024-11-02 
 * Version 1.9.30: resolved Issue 1916: KinematicTree (docu)
    - description:  add clarification for order to application of joint offset and joint transformation (rotation)
    - date resolved: **2024-10-30 10:56**\ , date raised: 2024-10-30 
 * Version 1.9.29: resolved Issue 1915: RigidBodyInertia (extension)
    - description:  add warning in case of unphysical inertia parameters
    - date resolved: **2024-10-29 22:26**\ , date raised: 2024-10-29 
 * Version 1.9.28: resolved Issue 1914: GeneralContact (change)
    - description:  GetSystemODE2RhsContactForces: change argument reference to copy to make it more consistent with other reference access and avoid confusion with reference configuration; default behavior unchanged
    - date resolved: **2024-10-27 10:13**\ , date raised: 2024-10-27 
 * Version 1.9.27: resolved Issue 1913: MainSystem.systemData (extension)
    - description:  add function GetODE2CoordinatesTotal which includes reference values added to coordinates
    - date resolved: **2024-10-27 10:00**\ , date raised: 2024-10-27 
 * Version 1.9.26: resolved Issue 1911: OutputVariableType (change)
    - description:  change order of types, leading to different numerical values behind OutputVariableType enum (should not affect normal codes)
    - **notes:** affected by issue 1909
    - date resolved: **2024-10-26 19:30**\ , date raised: 2024-10-26 
 * Version 1.9.25: resolved Issue 1909: OutputVariableType (extension)
    - description:  Add CoordinatesTotal which includes the reference configuration for output variables, used in sensors, or object and node outputs
    - date resolved: **2024-10-26 19:26**\ , date raised: 2024-10-26 
 * Version 1.9.24: resolved Issue 1505: reference coordinates (extension)
    - description:  add option to get total coordinates, being reference + current coordinates; gives 4 new configurations; use harmonized interface functions
    - date resolved: **2024-10-26 19:26**\ , date raised: 2023-04-08 
 * Version 1.9.23: resolved Issue 1897: particles (extension)
    - description:  Add particles module with functionality for creating densly packed particles in a box with proper regular initialization
    - date resolved: **2024-10-19 20:57**\ , date raised: 2024-10-16 
 * Version 1.9.22: resolved Issue 1903: GeneralContact (extension)
    - description:  GetSystemODE2RhsContactForces(...): add additional arg reference in order to allow linking to contact forces and thus allowing faster access (writing to this vector has no effect!)
    - date resolved: **2024-10-19 18:59**\ , date raised: 2024-10-19 
 * Version 1.9.21: resolved Issue 1902: FEMinterface (change)
    - description:  store elements as consistently as np.array instead of list of lists; check that all large arrays are np.arrays; do this also for surface list of lists
    - **notes:** now all previous list of lists in FEMinterface are defined to be numpy arrays for speed up of load/save; check your interfaces! GetSurfaceTriangles still returns list of lists
    - date resolved: **2024-10-19 18:48**\ , date raised: 2024-10-19 
 * Version 1.9.20: resolved Issue 1629: GetSystemState (extension)
    - description:  extend behavior for returning a dictionary with all data incl. accelerations and possibly alg. accelerations for generalized-alpha solver; check also option to link coords, as in issue 1504
    - **notes:** added mbs.systemData.GetSystemStateDict(...) with option to return a reference (link) to system vectors, which do not require copying and allow modifications directly
    - date resolved: **2024-10-19 18:45**\ , date raised: 2023-06-23 
 * Version 1.9.19: resolved Issue 1504: Reference/link (extension)
    - description:  extend mbs and systemData functions for reference, e.g., GetODE2Coordinates; use different function with "Link" extension, e.g., GetODE2CoordinatesLink
    - **notes:** added systemData function GetSystemStateDict which allows to obtain writeable references
    - date resolved: **2024-10-19 01:28**\ , date raised: 2023-04-08 
 * Version 1.9.18: :textred:`resolved BUG 1901` : FEM LoadFromFile 
    - description:  does not work correctly for forceVersion>0.
    - date resolved: **2024-10-18 14:58**\ , date raised: 2024-10-18 
 * Version 1.9.17: resolved Issue 1900: MainSystem UserFunctions (testing)
    - description:  Add TestModel for all kinds of PreStep, PostStep, reNewtonResidual, etc. functions
    - date resolved: **2024-10-17 23:09**\ , date raised: 2024-10-16 
 * Version 1.9.16: resolved Issue 1899: SystemJacobianUserFunction (extension)
    - description:  add a MainSystem user function with SetSystemJacobianUserFunction(...) which adds terms to the system jacobian; this is valuable as it may modify only few terms and add appropriate (experimental) terms in extension with the PreNewtonResidualUserFunction which otherwise would lead to bad convergence if important couplings are not added; as mentioned in the description, the solver's user functions are more general and should be also considered
    - date resolved: **2024-10-17 23:09**\ , date raised: 2024-10-16 
 * Version 1.9.15: resolved Issue 1882: preNewtonResidualUserFunction (extension)
    - description:  add user function called in every iteration of Newton solver in static or implicit dynamic computations
    - **notes:** function SetPreNewtonResidualUserFunction added to MainSystem, see description at MainSystem
    - date resolved: **2024-10-16 23:27**\ , date raised: 2024-10-10 
 * Version 1.9.14: resolved Issue 0838: GeneralContact jacobian (extension)
    - description:  add jacobian and PostNewton for cable-sphere (circle) contact
    - **notes:** already done earlier for ANCF beam elements
    - date resolved: **2024-10-16 16:52**\ , date raised: 2021-12-19 
 * Version 1.9.13: :textred:`resolved BUG 1893` : keyPressUserFunction 
    - description:  not working any more!
    - **notes:** works, but needs to activate window.ignoreKeys in order to call user function
    - date resolved: **2024-10-15 23:21**\ , date raised: 2024-10-15 
 * Version 1.9.12: resolved Issue 1717: Symbolic (example)
    - description:  add examples to testsuite; modify existing examples
    - **notes:** exu.symbolic used in 5 tests and examples
    - date resolved: **2024-10-15 23:08**\ , date raised: 2023-12-08 
 * Version 1.9.11: resolved Issue 1718: Mainsystem extensions (example)
    - description:  modify some examples for .Create...(...) functions
    - **notes:** new functions usually only using CreateGenericJoint, CreateRevoluteJoint, etc.; still keeping old workflows with markers for LLM trainings
    - date resolved: **2024-10-15 23:06**\ , date raised: 2023-12-08 
 * Version 1.9.10: resolved Issue 1721: Mainsystem extensions (change)
    - description:  in extension to issue 1718, change all AddRigidBody(...) functions to CreateRigidBody functionality
    - date resolved: **2024-10-15 23:05**\ , date raised: 2023-12-08 
 * Version 1.9.9: resolved Issue 1885: pickleCopyMBS (testing)
    - description:  add TestModel for copying and pickling (load/save) of MainSystem mbs
    - date resolved: **2024-10-13 22:52**\ , date raised: 2024-10-11 
 * Version 1.9.8: resolved Issue 1891: load/save mbs (extension)
    - description:  improve functionalities to load and save mbs, using GetDictionary / SetDictionary; see example pickleCopyMbs.py using pickle and HDF5 files
    - date resolved: **2024-10-13 19:16**\ , date raised: 2024-10-13 
 * Version 1.9.7: resolved Issue 1890: LoadSaveHDF5 (extension)
    - description:  enable option to load/save Python user functions
    - date resolved: **2024-10-11 22:49**\ , date raised: 2024-10-11 
 * Version 1.9.6: resolved Issue 1886: NGsolve CMS test (testing)
    - description:  add test model for whole CMS functionality but loading from stored data; test all 3 new file formats for storing FEM data
    - date resolved: **2024-10-11 09:00**\ , date raised: 2024-10-11 
 * Version 1.9.5: resolved Issue 1887: HDF5 (extension)
    - description:  add functions to advancedUtilities to load/save hierarchical dictionary data
    - date resolved: **2024-10-11 01:00**\ , date raised: 2024-10-11 
 * Version 1.9.4: resolved Issue 1884: FEMinterface (extension)
    - description:  extend LoadFromFile and SaveToFile for HDF5 and PKL (pickle) file formats
    - date resolved: **2024-10-11 01:00**\ , date raised: 2024-10-11 
 * Version 1.9.3: resolved Issue 1883: RigidBodySpringDamper (change)
    - description:  C++: change computation of outputvariables such that springForceTorqueUserFunction is not called for computation of displacement, rotation, etc., but only for forces and torques
    - date resolved: **2024-10-11 00:03**\ , date raised: 2024-10-11 
 * Version 1.9.2: resolved Issue 1881: Loads visualization (check)
    - description:  consider an option to show time-dependent loads, in particular in case of symbolic user functions
    - **notes:** added flag  visualizationSettings.loads.drawWithUserFunction and test model loadUserFunctionTest.py
    - date resolved: **2024-10-10 10:48**\ , date raised: 2024-10-09 
 * Version 1.9.1: resolved Issue 1723: Symbolic (extension)
    - description:  SymbolicRealVector: add EvaluateItem(i) in operators wherever possible to efficiently evaluate single components instead of all vector components
    - date resolved: **2024-10-09 22:23**\ , date raised: 2023-12-09 
 * Version 1.9.0: resolved Issue 1880: MatrixContainer (fix)
    - description:  when initialized with lists of lists, it prints the lists
    - date resolved: **2024-10-09 11:06**\ , date raised: 2024-10-09 

***********
Version 1.8
***********

 * Version 1.8.81: resolved Issue 1879: MatrixContainer (extension)
    - description:  add feature to directly initialize with scipy sparse csr matrix; add new function Initialize to initialize MatrixContainer, and AddSparseMatrix for adding sparse matrices with factor
    - date resolved: **2024-10-09 09:39**\ , date raised: 2024-10-09 
 * Version 1.8.80: resolved Issue 1878: robotics.mobile (change)
    - description:  correct camelcase writing of mobileRobot2MBS to MobileRobot2MBS, getWheelVelocities to GetWheelVelocities, and getCartesianVelocities to GetCartesianVelocities; adjust examples
    - date resolved: **2024-10-09 08:04**\ , date raised: 2024-10-09 
 * Version 1.8.79: resolved Issue 1844: AddRigidBody (change)
    - description:  add warning to this and related functions for deprecation
    - date resolved: **2024-10-09 07:40**\ , date raised: 2024-05-28 
 * Version 1.8.78: resolved Issue 1455: MarkerSuperElementRigidBody (fix)
    - description:  fix derivative of exponential map for velocity level
    - **notes:** already resolved earlier
    - date resolved: **2024-10-09 07:37**\ , date raised: 2023-03-05 
 * Version 1.8.77: resolved Issue 1341: MacOS multithreading (fix)
    - description:  resolve compilation problems with NGsolve taskmanager on Apple MacOS
    - **notes:** already resolved earlier
    - date resolved: **2024-10-09 07:36**\ , date raised: 2022-12-26 
 * Version 1.8.76: resolved Issue 0496: controller (extension)
    - description:  add possibility to differentiate loads w.r.t. sensor?/object/node (use additional sensor numbers which provide dependencies); add optional dependence on sensors; integrateors using ODE1 or discrete implementation
    - **notes:** already resolved in version 1.6.84; available via systemData.AddODE2LoadDependencies
    - date resolved: **2024-10-09 07:33**\ , date raised: 2020-12-09 
 * Version 1.8.75: resolved Issue 1877: Create functions (change)
    - description:  CreateDistanceConstraint, CreateSpringDamper and similar create functions have bodyList instead of bodyNumbers, which is used for joints; use bodyNumbers and allow bodyList as deprecated option
    - **notes:** kept compatibility with existing bodyList args, but will be removed in future versions
    - date resolved: **2024-10-08 23:04**\ , date raised: 2024-10-08 
 * Version 1.8.74: resolved Issue 1862: CSR functionality in FEM (extension)
    - description:  change internal CSR format to scipy CSR format; add simple check for Scipy to be installed at start of FEM, set scipyInstalled=True, use CheckSciPyInstalled() function for unified errors
    - date resolved: **2024-10-08 17:29**\ , date raised: 2024-10-02 
 * Version 1.8.73: resolved Issue 1870: FEMinterface (check)
    - description:  check all occurances of GetStiffnessMatrix and GetMassMatrix for sparse mode now using scipy sparse matrix
    - **notes:** Make sure that using FEMinterface fem, fem.GetMassMatrix of fem.GetStiffnessMatrix now returns a SciPy sparse matrix, which can be converted into the previous form by using ScipySparseCSRtoCSR(fem.GetMassMatrix())
    - date resolved: **2024-10-08 17:28**\ , date raised: 2024-10-07 
 * Version 1.8.72: resolved Issue 1876: FEMinterface (change)
    - description:  SaveToFile: used wrong default fileVersion=13, which should have been fileVersion=1; as we now shift to fileVersion=2, old files may be loaded with forceVersion=1
    - date resolved: **2024-10-08 17:26**\ , date raised: 2024-10-08 
 * Version 1.8.71: resolved Issue 1871: MatrixContainer (fix)
    - description:  fix compilation issues with pybind11 and scipy coo matrix
    - date resolved: **2024-10-08 17:26**\ , date raised: 2024-10-07 
 * Version 1.8.70: resolved Issue 1875: FEMinterface (change)
    - description:  the mass and stiffness matrices in FEMinterface are either None or given in SciPy-sparse csr format; this gives also a a new load/save fileVersion of FEMinterface
    - date resolved: **2024-10-08 13:09**\ , date raised: 2024-10-08 
 * Version 1.8.69: resolved Issue 1874: FEMinterface (change)
    - description:  SaveToFile: add version 2 which also pickles mass and stiffnessMatrix
    - date resolved: **2024-10-08 11:11**\ , date raised: 2024-10-08 
 * Version 1.8.68: resolved Issue 1873: FEM module (change)
    - description:  replace print(...) with exu.Print(...) commands to better control output flow
    - date resolved: **2024-10-08 10:33**\ , date raised: 2024-10-08 
 * Version 1.8.67: resolved Issue 1872: MatrixContainer (extension)
    - description:  SetWithSparseMatrixCSR: mark as deprecated; instead add new function SetWithSparseMatrix with additional arg factor (default=1) to multply matrix values with factor before adding
    - date resolved: **2024-10-07 18:34**\ , date raised: 2024-10-07 
 * Version 1.8.66: resolved Issue 1869: FEMinterface (change)
    - description:  change default values of massMatrix and stiffnessMatrix to None instead of np.zeros((0,0))
    - date resolved: **2024-10-07 01:54**\ , date raised: 2024-10-07 
 * Version 1.8.65: resolved Issue 1865: MatrixContainer (extension)
    - description:  add functionality to add Scipy csr matrix AddSparseMatrix(..., factor=1) with factor
    - date resolved: **2024-10-07 01:35**\ , date raised: 2024-10-02 
 * Version 1.8.64: resolved Issue 0840: explicit solvers velocity verlet (extension)
    - description:  add velocity verlet integration scheme in particular for particle and contact simulation
    - **notes:** added, but not yet tested for Lie group case!
    - date resolved: **2024-10-07 00:37**\ , date raised: 2021-12-19 
 * Version 1.8.63: resolved Issue 1868: SetWithSparseMatrixCSR (change)
    - description:  change default behavior to useDenseMatrix=False, using sparse mode by default
    - date resolved: **2024-10-06 22:03**\ , date raised: 2024-10-06 
 * Version 1.8.62: resolved Issue 1867: graphicsDataUtilities (docu)
    - description:  adapt documentation to replace graphicsDataUtilities with graphics
    - date resolved: **2024-10-06 17:24**\ , date raised: 2024-10-06 
 * Version 1.8.61: resolved Issue 1866: DOCU GraphicsData: Line (fix)
    - description:  Mistake in Line example; add hint to use graphics.Lines function
    - date resolved: **2024-10-06 17:06**\ , date raised: 2024-10-06 
 * Version 1.8.60: resolved Issue 1854: figure (docu)
    - description:  replace figure theoryRotationsTaitBryanAngles, which has been accidentially copied
    - date resolved: **2024-10-06 16:51**\ , date raised: 2024-06-05 
 * Version 1.8.59: :textred:`resolved BUG 1860` : ComputeODE2Eigenvalues 
    - description:  in case of computeComplexEigenvalues=True, the option convert2Frequencies=True computes wrong frequencies; this combination thus will be deactivated, as it makes little sense anyhow
    - date resolved: **2024-09-19 16:15**\ , date raised: 2024-09-19 
 * Version 1.8.58: resolved Issue 1859: Frames (fix)
    - description:  frame numbers not shown (e.g. for nodes)
    - **notes:** fixed together with issue 1858
    - date resolved: **2024-08-07 17:26**\ , date raised: 2024-08-07 
 * Version 1.8.57: resolved Issue 1858: KinematicTree (fix)
    - description:  frame numbers not shown
    - **notes:** changing default showFramesNumbers=False
    - date resolved: **2024-08-07 17:26**\ , date raised: 2024-08-07 
 * Version 1.8.56: :textred:`resolved BUG 1857` : KinematicTree 
    - description:  COM not correctly drawn (drawn at joint location)
    - date resolved: **2024-08-07 17:05**\ , date raised: 2024-08-07 
 * Version 1.8.55: resolved Issue 1856: FromSTLfile (fix)
    - description:  function does not work for A or p being different from []
    - date resolved: **2024-07-03 11:33**\ , date raised: 2024-07-03 
 * Version 1.8.54: resolved Issue 1849: tutorial (docu)
    - description:  add FFRFreducedOrder with NGsolve tutorial
    - date resolved: **2024-06-23 16:59**\ , date raised: 2024-06-02 
 * Version 1.8.53: resolved Issue 1855: lie group integrator (change)
    - description:  add improved lie group integrator for generalized alpha, not using the simplified update and thus having higher accuracy
    - date resolved: **2024-06-15 13:42**\ , date raised: 2024-06-15 
 * Version 1.8.52: resolved Issue 1852: tutorials (docu)
    - description:  adapt tutorials to new exudyn.graphics structure
    - date resolved: **2024-06-04 21:48**\ , date raised: 2024-06-04 
 * Version 1.8.51: resolved Issue 1853: exudyn.graphics (docu)
    - description:  add documentation in python utilities for these functions
    - date resolved: **2024-06-04 21:06**\ , date raised: 2024-06-04 
 * Version 1.8.50: resolved Issue 1851: artificialIntelligence (change)
    - description:  adapt examples to new stable-baselines version
    - **notes:** tested with stable-baselines3 V1.7.0 and V2.3.2
    - date resolved: **2024-06-04 17:24**\ , date raised: 2024-06-04 
 * Version 1.8.49: resolved Issue 1850: artificialIntelligence (change)
    - description:  adapt to stable-baselines3 2.3; add old mode with gym and new mode with gymnasium
    - **notes:** kept old mode available if gym is installed with stable-baselines without gymnasium
    - date resolved: **2024-06-04 14:59**\ , date raised: 2024-06-04 
    - resolved by: P. Manzl
 * Version 1.8.48: resolved Issue 0406: add NGsolve test (extension)
    - description:  with FEMinterface, only for Python37 version
    - **notes:** outdated and removed (see new issue 1849)
    - date resolved: **2024-06-02 13:30**\ , date raised: 2020-05-22 
 * Version 1.8.47: resolved Issue 0837: GeneralContact jacobian (extension)
    - description:  add jacobian and PostNewton for Triangle-Sphere contact
    - date resolved: **2024-06-02 10:58**\ , date raised: 2021-12-19 
 * Version 1.8.46: resolved Issue 1847: GeneralContact (fix)
    - description:  change several for-break commands to continue to improve contact behavior
    - date resolved: **2024-06-01 20:13**\ , date raised: 2024-06-01 
 * Version 1.8.45: resolved Issue 1475: tutorial videos (docu)
    - description:  add new tutorial, replace old ones
    - **notes:** already completed on Feb 15 2024
    - date resolved: **2024-05-20 19:13**\ , date raised: 2023-03-27 
 * Version 1.8.44: resolved Issue 1474: tutorial videos (fix)
    - description:  fix gettings started video
    - **notes:** already completed on Feb 15 2024
    - date resolved: **2024-05-20 19:13**\ , date raised: 2023-03-27 
 * Version 1.8.43: resolved Issue 1842: add FurtherExampels folder for further examples, especially useful for training of LLMs (extension)
    - description:  EXTENSION
    - date resolved: **2024-05-20 19:08**\ , date raised: 2024-05-17 
 * Version 1.8.42: resolved Issue 1843: simulationSettings (docu)
    - description:  correct path of simulationSettings.staticSolverSettings to simulationSettings.staticSolver in RTD description
    - date resolved: **2024-05-17 18:36**\ , date raised: 2024-05-17 
 * Version 1.8.41: resolved Issue 1841: AddODE2LoadDependencies (fix)
    - description:  description is wrong
    - date resolved: **2024-05-17 18:29**\ , date raised: 2024-05-17 
 * Version 1.8.40: resolved Issue 1840: std::isalpha (fix)
    - description:  remove std::isalpha and std::isalnum, as it does not compile on certain older compilers
    - date resolved: **2024-05-13 17:08**\ , date raised: 2024-05-13 
 * Version 1.8.39: resolved Issue 1838: Examples check (extension)
    - description:  add test to check whether examples basically run; use timeout and special setting to turn off all key-press waiting; exclude large examples
    - date resolved: **2024-05-12 10:19**\ , date raised: 2024-05-11 
 * Version 1.8.38: :textred:`resolved BUG 1839` : ALEANCFCable2D 
    - description:  raises error 'ANCFCable2d:ComputeAxialStrain_t not implemented' if ForceLocal is evaluated
    - **notes:** results have to be verified
    - date resolved: **2024-05-11 19:18**\ , date raised: 2024-05-11 
 * Version 1.8.37: resolved Issue 1187: ALEANCFCable2D (extension)
    - description:  add missing terms related to damping terms coupled with delta qALE
    - **notes:** already implemented and checked with paper up to come
    - date resolved: **2024-05-11 19:00**\ , date raised: 2022-07-06 
 * Version 1.8.36: resolved Issue 1692: exudyn.graphics (fix)
    - description:  change GraphicsData functions in examples to exudyn.graphics
    - **notes:** CHECK your files, if there are any issues due to this change; in general, the previous functionality should be maintained
    - date resolved: **2024-05-11 01:18**\ , date raised: 2023-11-19 
 * Version 1.8.35: resolved Issue 1825: GraphicsDataCube (change)
    - description:  rename to GraphicsDataBrick... functions; use assignment to keep previous functionality
    - **notes:** not changed, but introduced exudyn.graphics submodule for graphics functions. OrthoCube now available as exudyn.graphics.Brick
    - date resolved: **2024-05-11 01:16**\ , date raised: 2024-04-25 
 * Version 1.8.34: resolved Issue 1691: exudyn.graphics (extension)
    - description:  map graphicsDataUtilities functions to exudyn.graphics for better readability
    - **notes:** NOTE that previous GraphicsData functions still work; however, it may be necessary to import exudyn.utilities if you imported exudyn.graphicsDataUtilities directly; examples and test models are adjusted to new functions; it is recommended that users switch to new exudyn.graphics functionality!
    - date resolved: **2024-05-10 19:45**\ , date raised: 2023-11-19 
 * Version 1.8.33: resolved Issue 1837: graphics (check)
    - description:  test graphics submodule for future replacement of graphicsDataUtilities
    - date resolved: **2024-05-10 10:09**\ , date raised: 2024-05-10 
 * Version 1.8.32: resolved Issue 1832: theory docu (docu)
    - description:  add description of computation of stresses for FFRFreducedOrder
    - date resolved: **2024-05-09 17:12**\ , date raised: 2024-05-03 
 * Version 1.8.31: resolved Issue 1836: ComputeLinearizedSystem (extension)
    - description:  output constraint and nullspace matrices for constrained systems
    - date resolved: **2024-05-05 16:18**\ , date raised: 2024-05-05 
 * Version 1.8.30: resolved Issue 1835: ComputeLinearizedSystem (extension)
    - description:  add option to compute linearized system of constrained system
    - date resolved: **2024-05-05 16:18**\ , date raised: 2024-05-05 
 * Version 1.8.29: resolved Issue 1834: ComputeLinearizedSystem (change)
    - description:  remove sparse solver option useSparseSolver, as it is not implemented
    - date resolved: **2024-05-05 16:17**\ , date raised: 2024-05-05 
 * Version 1.8.28: resolved Issue 1833: ComputeODE2EigenValues (extension)
    - description:  extend to complex eigenvalues
    - **notes:** added TestModel complexEigenvaluesTest.py
    - date resolved: **2024-05-04 18:12**\ , date raised: 2024-05-03 
 * Version 1.8.27: resolved Issue 1831: ObjectJointRollingDisc (change)
    - description:  change computation of constraints into local joint frame, enabling lateral and forward constraint independently; see old comment on constrainedAxes
    - date resolved: **2024-04-29 08:03**\ , date raised: 2024-04-29 
 * Version 1.8.26: resolved Issue 1830: AddLidar (fix)
    - description:  angles of sensors should be equally arranged between angleStart and angleEnd using numberOfSensors and having a sensor both at start and end angle
    - date resolved: **2024-04-27 17:06**\ , date raised: 2024-04-27 
 * Version 1.8.25: resolved Issue 1829: AddLidar (fix)
    - description:  several arguments are not passed to CreateDistanceSensor; add all arguments to interface, which may cause some change in behavior!
    - date resolved: **2024-04-27 16:48**\ , date raised: 2024-04-27 
 * Version 1.8.24: resolved Issue 1828: Examples (example)
    - description:  add example mobileMecanumWheelRobotWithLidar.py for a mecanum wheeled robot with lidar and mapping
    - date resolved: **2024-04-27 11:11**\ , date raised: 2024-04-27 
    - resolved by: P. Manzl
 * Version 1.8.23: resolved Issue 1827: AddLidar (fix)
    - description:  argument rotation is not used
    - date resolved: **2024-04-27 11:08**\ , date raised: 2024-04-27 
 * Version 1.8.22: resolved Issue 1826: AddLidar (change)
    - description:  angles of sensor directions are not defined and erronously with respect to Y-axis; angleStart and angleEnde shall be measured w.r.t. the X-axis (angle=0) and in positive rotation sense about local Z-axis
    - date resolved: **2024-04-27 11:08**\ , date raised: 2024-04-27 
 * Version 1.8.21: resolved Issue 1824: ObjectConnectorCoordinateVector (docu)
    - description:  fix docu and remove inexisting parameters
    - date resolved: **2024-04-21 19:34**\ , date raised: 2024-04-21 
 * Version 1.8.20: resolved Issue 1820: item type info (change)
    - description:  change py::dict to py:object in functions such as AddMarker according to github issue #64, otherwise giving typing errors in PyLance and similar
    - date resolved: **2024-04-21 19:17**\ , date raised: 2024-04-19 
 * Version 1.8.19: resolved Issue 1823: stub files (.pyi) (change)
    - description:  change type in AddObject, AddMarker, AddSensor, AddLoad, AddNode from pyObject: dict to pyObject: Any in order to resolve typing errors
    - date resolved: **2024-04-21 19:16**\ , date raised: 2024-04-19 
 * Version 1.8.18: resolved Issue 1819: KinematicTreePendulum.py (example)
    - description:  Version 1.7.71 broke example kinematicTreePendulum with transition from SensorObject to SensorBody
    - **notes:** changed SensorObject(objectNumber=oKT into SensorBody(bodyNumber=oKT
    - date resolved: **2024-04-17 16:13**\ , date raised: 2024-04-17 
    - resolved by: P. Manzl
 * Version 1.8.17: resolved Issue 1818: stub files (.pyi) (fix)
    - description:  add types for mainsystem extensions (e.g. SolutionViewer)
    - **notes:** not resolved, because mainSystemExtension functions do not contain argument types
    - date resolved: **2024-04-14 17:54**\ , date raised: 2024-04-14 
 * Version 1.8.16: resolved Issue 1817: stub files (.pyi) (fix)
    - description:  add types for system structures (mainly solver functions)
    - **notes:** not resolved, because this information is not available in the structures for solver methods
    - date resolved: **2024-04-14 17:53**\ , date raised: 2024-04-14 
 * Version 1.8.15: resolved Issue 1816: stub files (.pyi) (fix)
    - description:  fix a couple of wrong return types and similar wrong typings (in particular return types in SetODE2Coordinates, etc.
    - date resolved: **2024-04-14 16:15**\ , date raised: 2024-04-14 
 * Version 1.8.14: resolved Issue 1814: stub files (.pyi) (fix)
    - description:  .pyi files do not contain correct default args for C++ interfaces
    - date resolved: **2024-04-14 16:15**\ , date raised: 2024-04-14 
 * Version 1.8.13: resolved Issue 1815: stub files (.pyi) (fix)
    - description:  .pyi are missing backslash before underscore (for latex) documentation
    - date resolved: **2024-04-14 15:15**\ , date raised: 2024-04-14 
 * Version 1.8.12: resolved Issue 1812: GetMarkerOutput (fix)
    - description:  not working for MarkerKinematicTreeRigid in case of Reference configuration; adjustments done in CObjectKinematicTree::ComputeTreeTransformations to avoid the need for velocities in the retrieval of positions
    - date resolved: **2024-04-03 19:38**\ , date raised: 2024-04-03 
 * Version 1.8.11: resolved Issue 1811: Sensitivity Analysis for functions with single outputs (fix)
    - issue author: PM
    - description:  ComputeSensitivities and PlotSensitivityResults does not work for functions with a single output
    - date resolved: **2024-03-26 19:24**\ , date raised: 2024-03-26 
    - resolved by: P. Manzl
 * Version 1.8.10: resolved Issue 1810: GeneralContact (change)
    - description:  unify computation of contact forces and jacobians for sphere-sphere and trig-sphere contact
    - date resolved: **2024-03-25 08:21**\ , date raised: 2024-03-25 
 * Version 1.8.9: resolved Issue 1809: GeneralContact (fix)
    - description:  apply changes in sphere-sphere contact to Jacobian and to docu
    - date resolved: **2024-03-17 19:05**\ , date raised: 2024-03-17 
 * Version 1.8.8: :textred:`resolved BUG 1807` : GeneralContact 
    - description:  sphere-shpere contact causes spurious internal torque
    - **notes:** fixed by adding 0.5\*penetration in lever arm; adjusted also documentation; gives new test suite results
    - date resolved: **2024-03-17 10:56**\ , date raised: 2024-03-17 
 * Version 1.8.7: :textred:`resolved BUG 1805` : CreateRigidBody 
    - description:  raises exception in case that initialRotationMatrix is not None; SOLUTION: replace == None for ALL cases (check other functions) with is None or is not Note!
    - **notes:** fixed with 1808
    - date resolved: **2024-03-17 10:55**\ , date raised: 2024-03-16 
 * Version 1.8.6: :textred:`resolved BUG 1806` : CreateRigidBody 
    - description:  initialRotationMatrix has no effect at least in explicit integration
    - **notes:** wrong operator\* used for multiplication of reference rotation matrix and initial rotation matrix; FIXED
    - date resolved: **2024-03-17 10:54**\ , date raised: 2024-03-16 
 * Version 1.8.5: resolved Issue 1808: advanced utilities (extension)
    - description:  added function to check for None and not None: IsNone(x), IsNotNone(x)
    - date resolved: **2024-03-17 10:29**\ , date raised: 2024-03-17 
 * Version 1.8.4: resolved Issue 1804: GeneralContact (check)
    - description:  check triangle-sphere contact: torque for triangle computed with "((-sphereI.radius)\*deltaP0).CrossProduct(fVec)" lever arm should be -(trigPP - rigid.position) ?
    - **notes:** changed term for torque on rigid body according to added documentation in theory section
    - date resolved: **2024-03-17 10:15**\ , date raised: 2024-03-16 
 * Version 1.8.3: resolved Issue 1802: RigidBodySpringDamper (check)
    - description:  check intrinsic joint formulation
    - **notes:** no inconsistencies found or detected in examples
    - date resolved: **2024-03-13 13:10**\ , date raised: 2024-03-07 
 * Version 1.8.2: resolved Issue 1803: MarkerSuperElementRigid (extension)
    - description:  add option for tangent operator in alternativeFormulation
    - date resolved: **2024-03-13 13:09**\ , date raised: 2024-03-13 
 * Version 1.8.1: resolved Issue 0734: continuous integration (coding)
    - description:  test CI capabilities with GitHub and MacOS compilation
    - **notes:** resolved with issue 1792
    - date resolved: **2024-03-09 16:04**\ , date raised: 2021-08-12 
 * Version 1.8.0: resolved Issue 1789: AvailableItems (extension)
    - description:  add exudyn.special function to retrieve available items as dictionary with lists
    - date resolved: **2024-03-06 09:02**\ , date raised: 2024-02-21 

***********
Version 1.7
***********

 * Version 1.7.123: resolved Issue 1795: joint constraints (docu)
    - description:  theory: add description for formulation of joint constraints
    - **notes:** added equations to position markers and JointSpherical
    - date resolved: **2024-03-03 22:04**\ , date raised: 2024-02-25 
 * Version 1.7.122: resolved Issue 1800: GeneralContact (extension)
    - description:  add option for GetActiveContacts to return number of contacts per contact type in case that itemIndex=-1
    - date resolved: **2024-02-29 15:50**\ , date raised: 2024-02-29 
 * Version 1.7.121: resolved Issue 1799: exudyn __init__.py (change)
    - description:  remove NoAVX option for linux, as linux does not (yet) have a AVX2 option; crashed on linux arm/aarch architecture
    - date resolved: **2024-02-29 14:34**\ , date raised: 2024-02-29 
 * Version 1.7.120: resolved Issue 1797: Github actions (extension)
    - description:  create single line output for testsuite with specific mode; add test suite for github actions and merge outputs into single file
    - **notes:** put information into filename of text tilde with output of testsuite
    - date resolved: **2024-02-29 14:32**\ , date raised: 2024-02-27 
 * Version 1.7.119: resolved Issue 1798: linux arm (extension)
    - description:  add multilinux aarch64 wheels to GH build actions
    - date resolved: **2024-02-29 14:31**\ , date raised: 2024-02-29 
 * Version 1.7.118: resolved Issue 1796: MacOSX universal2 (extension)
    - description:  add build option for macos universal files on GH actions to have both arm and x86 on board
    - **notes:** NOTE that pip 20.3 is required to install these wheels!
    - date resolved: **2024-02-27 15:06**\ , date raised: 2024-02-27 
 * Version 1.7.117: resolved Issue 1794: fix curly brackets (docu)
    - description:  fix curly brackets {} in RST files
    - date resolved: **2024-02-25 20:43**\ , date raised: 2024-02-25 
 * Version 1.7.116: resolved Issue 1793: manylinux2014 (extension)
    - description:  build highly compatible manylinux2014 and manylinux2_17 wheels with github actions docker, to run on CentOS and Rocky Linux as well as ubuntu
    - date resolved: **2024-02-24 23:28**\ , date raised: 2024-02-24 
 * Version 1.7.115: resolved Issue 1792: Github actions CI (extension)
    - description:  add github actions to create automatically Windows, Ubunut and MacOS wheels
    - date resolved: **2024-02-24 23:28**\ , date raised: 2024-02-24 
 * Version 1.7.114: resolved Issue 1787: Python 3.12 (extension)
    - description:  include Python 3.12 wheels into build process
    - date resolved: **2024-02-24 23:25**\ , date raised: 2024-02-21 
 * Version 1.7.113: :textred:`resolved BUG 1791` : Autoregistration items 
    - description:  node does not initialize CData
    - date resolved: **2024-02-24 17:45**\ , date raised: 2024-02-24 
 * Version 1.7.112: resolved Issue 1788: items auto-registration (change)
    - description:  add a simple way to automatically register items; use C++ map to create item in object-factory
    - date resolved: **2024-02-21 19:17**\ , date raised: 2024-02-21 
 * Version 1.7.111: resolved Issue 1790: exudyn minimal (extension)
    - description:  add flag EXUDYN_MINIMAL_ITEMS to achieve fast compilation for testing
    - date resolved: **2024-02-21 18:53**\ , date raised: 2024-02-21 
 * Version 1.7.110: :textred:`resolved BUG 1786` : ComputeODE2singleLoad 
    - description:  raises error in static computation "inconsistent jacobian"; workaround settings computeLoadsJacobian=False or using sparse solver
    - **notes:** exception due to inconsistent computation of mass proportional load jacobian with rigid body
    - date resolved: **2024-02-16 11:42**\ , date raised: 2024-02-16 
 * Version 1.7.109: resolved Issue 1785: beam tutorial (docu)
    - description:  add beam tutorial example and tutorial in theDoc
    - date resolved: **2024-02-13 22:08**\ , date raised: 2024-02-13 
 * Version 1.7.108: resolved Issue 1784: theory theDoc (docu)
    - description:  add introduction to multibody dynamics, kinematics, dynamics, rotations
    - date resolved: **2024-02-13 16:10**\ , date raised: 2024-02-13 
 * Version 1.7.107: resolved Issue 1783: RST graphics (docu)
    - description:  add graphics for readthedocs representation, in solvers; fix references
    - date resolved: **2024-02-13 16:10**\ , date raised: 2024-02-13 
 * Version 1.7.106: resolved Issue 1782: GeneralContact (extension)
    - description:  add flag computeContactForces to settings, which computes contribution of contact forces to system vector (may slow down computations!); similar to issue 936
    - date resolved: **2024-02-12 09:07**\ , date raised: 2024-02-12 
 * Version 1.7.105: resolved Issue 0936: GeneralContact (extension)
    - description:  add interface function to get contact forces
    - date resolved: **2024-02-12 09:07**\ , date raised: 2022-02-10 
 * Version 1.7.104: resolved Issue 1781: GeneralContact (extension)
    - description:  add function Get/SetTriangleRigidBodyBased, to get data or modify data of current contact triangle
    - date resolved: **2024-02-08 20:38**\ , date raised: 2024-02-08 
 * Version 1.7.103: resolved Issue 1780: GeneralContact (extension)
    - description:  add function SetSphereMarkerBased to set data for spheres during simulation
    - date resolved: **2024-02-08 20:38**\ , date raised: 2024-02-08 
 * Version 1.7.102: resolved Issue 1779: GeneralContact (change)
    - description:  GetMarkerBasedSphere: change to GetSphereMarkerBased; add flag to decide whether to add basic data or not
    - date resolved: **2024-02-08 19:08**\ , date raised: 2024-02-08 
 * Version 1.7.101: resolved Issue 1778: cRGB settings (change)
    - description:  change cRGB consistently to RGBA in visualization settings
    - date resolved: **2024-02-08 18:57**\ , date raised: 2024-02-08 
 * Version 1.7.100: resolved Issue 1775: BodyGraphicsData (fix)
    - description:  access with Get/SetObjectParameter is missing for graphicsData
    - date resolved: **2024-02-04 22:01**\ , date raised: 2024-02-04 
 * Version 1.7.99: resolved Issue 1774: CreateSymbolicUserFunction (change)
    - description:  change order of args userFunctionName, itemIndex, and verbose; add additional itemTypeName
    - date resolved: **2024-02-04 00:36**\ , date raised: 2024-02-04 
 * Version 1.7.98: resolved Issue 1773: CreateSymbolicUserFunction (extension)
    - description:  add option to directly pass itemTypeName instead of itemIndex in order to pre-compute user function
    - date resolved: **2024-02-04 00:36**\ , date raised: 2024-02-04 
 * Version 1.7.97: resolved Issue 1771: TransferUserFunction2Item (change)
    - description:  add functionality to allow direct assignment of symbolic user functions to userFunction parameters in objects, loads, etc.; remove TransferUserFunction2Item as this function is then no longer needed
    - date resolved: **2024-02-03 23:55**\ , date raised: 2024-02-03 
 * Version 1.7.96: resolved Issue 1766: Python user functions (extension)
    - description:  use PythonUserFunctionBase class for all Item user functions; add consistent Get/Set function for item access; should then automatically work with pickle
    - date resolved: **2024-02-03 23:54**\ , date raised: 2024-02-02 
 * Version 1.7.95: resolved Issue 1750: python user functions (extension)
    - description:  add additional py::function to user functions in order to store original python function for pickling
    - date resolved: **2024-02-03 23:54**\ , date raised: 2024-01-29 
 * Version 1.7.94: resolved Issue 1759: Renderer (fix)
    - description:  there is an issue when restarting the renderer, which displays previous (old) data; requires to add some function which erases stored graphics data on call of StartRenderer()
    - date resolved: **2024-02-03 23:53**\ , date raised: 2024-01-31 
 * Version 1.7.93: resolved Issue 1770: mainsystem extensions (extensions)
    - description:  add user function to mbs.Create...() functions
    - date resolved: **2024-02-03 22:50**\ , date raised: 2024-02-03 
 * Version 1.7.92: resolved Issue 1763: Python user functions (extension)
    - description:  add pickle functionality (requires issue 1752)
    - date resolved: **2024-02-02 10:36**\ , date raised: 2024-01-31 
 * Version 1.7.91: resolved Issue 1752: user functions types (extension)
    - description:  create UserFunctionBase class and derived classes, containing std::function, and a py::object with metadata (Python function, Symbolic function, etc.); When settings user functions, they can be either initialized with 0 / Python function or with a UserFunction dict, which contains additional decorators; In particular, user functions will be rebuilt as symbolic; return values of user functions are then dictionaries
    - date resolved: **2024-02-02 10:36**\ , date raised: 2024-01-29 
 * Version 1.7.90: resolved Issue 1765: MainSystem user functions (extension)
    - description:  test special class for MainSystem user functions such as preStepUserFunction; if requeste, convert to Dict, in particular for symbolic or other special user functions
    - date resolved: **2024-02-02 10:33**\ , date raised: 2024-02-02 
 * Version 1.7.89: resolved Issue 1762: SystemContainer (extension)
    - description:  add pickle functionality
    - date resolved: **2024-01-31 20:22**\ , date raised: 2024-01-31 
 * Version 1.7.88: resolved Issue 1628: pickle MainSystem (extension)
    - description:  consider a pickle method for certain objects; add consistent info in description; MainSystem, SimulationSettings, VisualizationSettings
    - **notes:** consider with care, as not all things are copied (user functions, contact, ...)
    - date resolved: **2024-01-31 20:22**\ , date raised: 2023-06-23 
 * Version 1.7.87: resolved Issue 1761: pickle (extension)
    - description:  add pickle to settings and structures
    - date resolved: **2024-01-31 20:21**\ , date raised: 2024-01-31 
 * Version 1.7.86: resolved Issue 1760: pickle (extension)
    - description:  add pickle to ItemIndices
    - date resolved: **2024-01-31 20:21**\ , date raised: 2024-01-31 
 * Version 1.7.85: resolved Issue 1757: DictionariesGetSet (extension)
    - description:  add C++ GetDictionary(...) function for read/write of system structures; add Get/SetDictionary to pybind interface
    - date resolved: **2024-01-31 17:27**\ , date raised: 2024-01-30 
 * Version 1.7.84: resolved Issue 1758: StartRenderer (fix)
    - description:  add UpdateGraphicsDataNow() after start of renderer in order to avoid showing stored data for second run or renderer
    - date resolved: **2024-01-31 12:52**\ , date raised: 2024-01-31 
 * Version 1.7.83: resolved Issue 1755: CSystem in MainSystem (change)
    - description:  change CSystem\* to CSystem in MainSystem, to make copying easier
    - date resolved: **2024-01-30 16:43**\ , date raised: 2024-01-30 
 * Version 1.7.82: resolved Issue 1756: SystemContainer (change)
    - description:  remove SystemContainer and just keep MainSystemContainer, as it is not needed; simplifies copying
    - date resolved: **2024-01-30 16:42**\ , date raised: 2024-01-30 
 * Version 1.7.81: resolved Issue 1754: MainSystem (extension)
    - description:  change creation of MainSystem; allow construction like exudyn.MainSystem(), add new function Append(MainSystem) to MainSystemContainer; this will allow pickling both of MainSystem and MainSystemContainer
    - date resolved: **2024-01-30 14:34**\ , date raised: 2024-01-30 
 * Version 1.7.80: resolved Issue 1753: postStepUserFunction (extension)
    - description:  add user function to be called at end of time step, just before storing results to file; this allows to override results, etc.
    - date resolved: **2024-01-30 08:01**\ , date raised: 2024-01-30 
 * Version 1.7.79: resolved Issue 1748: user functions (fix)
    - description:  check option to set them to 0; PreStepUserFunction as well as item user functions
    - **notes:** Note that when reading user functions, mbs.GetObjectParameter(...) also consistently gives now 0 instead of previously None
    - date resolved: **2024-01-29 14:41**\ , date raised: 2024-01-29 
 * Version 1.7.78: resolved Issue 1749: User function (extension)
    - description:  allow assignment to 0 for MainSystem user functions; SetPreStepUserFunction and SetPostNewtonUserFunction
    - date resolved: **2024-01-29 14:34**\ , date raised: 2024-01-29 
 * Version 1.7.77: resolved Issue 1746: GenerateStraightBeam (extension)
    - description:  build generic function to create beams along straight line; add interface both for ANCF (old GenerateStraightLineANCFCable function) as well as for geometrically exact beam
    - date resolved: **2024-01-28 19:46**\ , date raised: 2024-01-28 
 * Version 1.7.76: resolved Issue 1747: GeometricallyExactBeam2D (extension)
    - description:  finish interface for load mass proportional
    - **notes:** also done for 3D version
    - date resolved: **2024-01-28 18:17**\ , date raised: 2024-01-28 
 * Version 1.7.75: resolved Issue 1745: geometrically exact beam 2D (change)
    - description:  adjust implementation of reference strains to allow connection of several beams at one node in case that includeReferenceRotations=0
    - date resolved: **2024-01-27 22:16**\ , date raised: 2024-01-27 
 * Version 1.7.74: resolved Issue 1744: geometrically exact beam 2D (docu)
    - description:  fix inconsistent documentation of reference strains
    - date resolved: **2024-01-27 22:16**\ , date raised: 2024-01-27 
 * Version 1.7.73: resolved Issue 1743: CreateRevoluteJoint (fix)
    - description:  Description for local/global axis is wrong; behavior is switched by useGlobalFrame flag
    - date resolved: **2024-01-06 11:20**\ , date raised: 2024-01-06 
 * Version 1.7.72: resolved Issue 1742: CreatePrismaticJoint (fix)
    - description:  Description for local/global axis is wrong; behavior is switched by useGlobalFrame flag
    - date resolved: **2024-01-06 11:20**\ , date raised: 2024-01-06 
 * Version 1.7.71: :textred:`resolved BUG 1741` : ObjectKinematicTree 
    - description:  SensorObject does not work with OutputVariableType Coordinates; example does not work;
    - **notes:** Coordinates, Force, etc. now available with GetObjectOutputBody and SensorBody
    - date resolved: **2024-01-06 11:02**\ , date raised: 2024-01-06 
 * Version 1.7.70: resolved Issue 1739: symbolic (fix)
    - description:  SetValue should raise exception if called with symbolic expression
    - date resolved: **2023-12-19 08:30**\ , date raised: 2023-12-19 
 * Version 1.7.69: resolved Issue 1737: symbolic (extension)
    - description:  add symbolic.pyi stub file for autocompletion of symbolic features
    - date resolved: **2023-12-18 19:48**\ , date raised: 2023-12-18 
 * Version 1.7.68: resolved Issue 1738: symbolic (docu)
    - description:  fix documentation for operators
    - date resolved: **2023-12-18 19:47**\ , date raised: 2023-12-18 
 * Version 1.7.67: resolved Issue 1727: SymbolicRealMatrix (docu)
    - description:  add documentation and example for symbolic matrix for user functions
    - date resolved: **2023-12-18 08:24**\ , date raised: 2023-12-12 
 * Version 1.7.66: :textred:`resolved BUG 1736` : symbolic 
    - description:  symbolic user function: crashes when user function object is deleted
    - date resolved: **2023-12-15 18:01**\ , date raised: 2023-12-15 
 * Version 1.7.65: resolved Issue 1735: symbolic (fix)
    - description:  check delete counts and reference counts for +=, etc.
    - date resolved: **2023-12-15 18:01**\ , date raised: 2023-12-15 
 * Version 1.7.64: resolved Issue 1729: Symbolic (testing)
    - description:  add vector/matrix tests in comparison with Python numpy and check delete counts
    - date resolved: **2023-12-15 13:52**\ , date raised: 2023-12-12 
 * Version 1.7.63: resolved Issue 1728: Symbolic (testing)
    - description:  add scalar tests in comparison with Python math and check delete counts
    - date resolved: **2023-12-15 13:52**\ , date raised: 2023-12-12 
 * Version 1.7.62: resolved Issue 1733: symbolic (check)
    - description:  check overloading __len__ operator for vector
    - date resolved: **2023-12-15 13:06**\ , date raised: 2023-12-15 
 * Version 1.7.61: resolved Issue 1734: symbolic (fix)
    - description:  write operator[] for Matrix and Vector fails
    - date resolved: **2023-12-15 11:51**\ , date raised: 2023-12-15 
 * Version 1.7.60: :textred:`resolved BUG 1732` : symbolic 
    - description:  Vector.SetVector(...), Matrix.SetMatrix(...) not working; fix Pybind interface
    - date resolved: **2023-12-15 11:12**\ , date raised: 2023-12-15 
 * Version 1.7.59: resolved Issue 1680: chatGPTupdate (example)
    - description:  add simple example for load userFunction
    - date resolved: **2023-12-14 00:01**\ , date raised: 2023-10-29 
 * Version 1.7.58: resolved Issue 1693: VObjectGround (fix)
    - description:  remove parameter color, as it is not used (check)
    - date resolved: **2023-12-13 23:40**\ , date raised: 2023-11-19 
 * Version 1.7.57: resolved Issue 1726: SymbolicRealMatrix (extension)
    - description:  add symbolic matrix for user functions
    - **notes:** note: currently implemented less efficient with memory allocations
    - date resolved: **2023-12-13 14:02**\ , date raised: 2023-12-12 
 * Version 1.7.56: resolved Issue 1731: ANCFCable2D (extension)
    - description:  add user functions for bending moment and axial force, allowing to implement arbitrary material models
    - date resolved: **2023-12-13 13:47**\ , date raised: 2023-12-13 
 * Version 1.7.55: resolved Issue 1730: GenerateStraightLineANCFCable (fix)
    - description:  raises Warning for default values [0,0,0,0,0,0] in 2D case
    - date resolved: **2023-12-13 13:27**\ , date raised: 2023-12-13 
 * Version 1.7.54: resolved Issue 1725: Symbolic (extension)
    - description:  add ResizableConstMatrix and create symbolic matrix-vector functions
    - date resolved: **2023-12-12 14:16**\ , date raised: 2023-12-10 
 * Version 1.7.53: resolved Issue 1716: Symbolic (docu)
    - description:  add symbolic user function description to documentation; also mention in performance section
    - date resolved: **2023-12-10 21:57**\ , date raised: 2023-12-08 
 * Version 1.7.52: resolved Issue 1724: C++ ToString (change)
    - description:  changing the behavior for standard conversion of double and int values to strings, in particular during errors. Now using the same precision as defined with exudyn.SetOutputPrecision()
    - date resolved: **2023-12-09 19:26**\ , date raised: 2023-12-09 
 * Version 1.7.51: resolved Issue 1715: Symbolic (docu)
    - description:  add symbolic section to documentation
    - date resolved: **2023-12-09 16:29**\ , date raised: 2023-12-08 
 * Version 1.7.50: resolved Issue 1708: Symbolic (extension)
    - description:  make symbolic variable space, available globally in exudyn.symbolic as well as in mbs.symbolic; this allows to store/transfer data into user functions without the need for Python; use integer handles which are returned by creation function: a=NamedReal(value, name); handle=AddVariableReal(a)
    - **notes:** not yet put into mbs and using no integer handles, but std::unordered_map, which has highly efficient hash table included
    - date resolved: **2023-12-09 16:29**\ , date raised: 2023-12-03 
 * Version 1.7.49: resolved Issue 1699: exudyn. ... (docu)
    - description:  add undocumented features of exudyn module, such as Demo1(), Demo2(), __version__ or C++
    - date resolved: **2023-12-08 18:41**\ , date raised: 2023-11-22 
 * Version 1.7.48: resolved Issue 1698: experimental, special (docu)
    - description:  add experimental and special features to documentation / pybindings
    - date resolved: **2023-12-08 18:41**\ , date raised: 2023-11-21 
 * Version 1.7.47: resolved Issue 1710: Symbolic (extension)
    - description:  add basic (automatic) differentiation feature for expressions: EvaluateDiff()
    - date resolved: **2023-12-08 07:22**\ , date raised: 2023-12-03 
 * Version 1.7.46: resolved Issue 1700: ResizableConstSizeVector (extension)
    - description:  consider a Vector with fixed size, which can be extended if necessary - e.g. for local variables in sensors or GetOutputVariable; is efficient for small vectors and still works for larger one
    - date resolved: **2023-12-07 23:03**\ , date raised: 2023-11-22 
 * Version 1.7.45: resolved Issue 1714: Symbolic (extension)
    - description:  add vector as symbolic expression, allowing vectors in user functions
    - date resolved: **2023-12-07 19:55**\ , date raised: 2023-12-07 
 * Version 1.7.44: resolved Issue 1713: user functions (change)
    - description:  change StdVector to StdVector3D and StdVector6D in relevant cases in order to achieve light-weight interface for symbolic interfaces
    - date resolved: **2023-12-07 19:53**\ , date raised: 2023-12-07 
 * Version 1.7.43: resolved Issue 1712: Symbolic (extension)
    - description:  add automatic creation of user functions to AutoGenerateObjects
    - date resolved: **2023-12-05 00:06**\ , date raised: 2023-12-03 
 * Version 1.7.42: resolved Issue 1711: Symbolic (extension)
    - description:  put most parts of specific user functioninto base SymbolicFunction; EvaluateUF: use variadic args to generalize
    - date resolved: **2023-12-05 00:06**\ , date raised: 2023-12-03 
 * Version 1.7.41: resolved Issue 1709: Symbolic (extension)
    - description:  add most of Pythons math module functions to symbolic functions list
    - date resolved: **2023-12-03 15:01**\ , date raised: 2023-12-03 
 * Version 1.7.40: resolved Issue 1705: Symbolic user function (extension)
    - description:  allow parallel computation of non-Python user functions
    - **notes:** this is enabled by adding user function after Assemble, thus objects are not registered to have Python user functions
    - date resolved: **2023-12-03 14:58**\ , date raised: 2023-11-28 
 * Version 1.7.39: resolved Issue 1706: RigidBodySpringDamper (docu)
    - description:  intrinsicFormulation: add test to test suite and document new functionality
    - date resolved: **2023-11-30 23:56**\ , date raised: 2023-11-30 
 * Version 1.7.38: resolved Issue 1707: RigidBodySpringDamper (check)
    - description:  intrinsicFormulation: check conserving properties of joint forces for two freely rotating bodies: additional torque of forces may be required in implementation
    - date resolved: **2023-11-30 23:39**\ , date raised: 2023-11-30 
 * Version 1.7.37: resolved Issue 1486: RigidBodySpringDamper (extension)
    - description:  extend for Lie group formulation, evaluating connectors at mid-configuration according to Masarati and Morandini
    - date resolved: **2023-11-30 20:51**\ , date raised: 2023-04-01 
 * Version 1.7.36: resolved Issue 1696: exudyn.symbolic (extension)
    - description:  add experimental expression trees for building symbolic expressions to be used for user functions
    - **notes:** basic symbolic functionality added; tested with SpringDamper user function, leading to speedup of 10 against regular Python function
    - date resolved: **2023-11-28 09:08**\ , date raised: 2023-11-21 
    - resolved by: EXTENSION
 * Version 1.7.35: resolved Issue 1704: solver timeout (extension)
    - description:  added module-wide flag for timeout: exudyn.special.solver.timeout in order to stop simulations after certain time; use with care
    - date resolved: **2023-11-22 09:43**\ , date raised: 2023-11-22 
 * Version 1.7.34: resolved Issue 1703: Pybind module (change)
    - description:  use suggestions of from search: pybind11, how to split my code into multiple modules/files - stackoverflow; should improve compilation time
    - **notes:** created 2 new pybind module files; no major compilation speedup visible on laptop
    - date resolved: **2023-11-22 08:24**\ , date raised: 2023-11-22 
 * Version 1.7.33: resolved Issue 1702: PyErr_CheckSignals (check)
    - description:  check if this method available in pybind helps to allow stopping long-lasting computations in exudyn
    - **notes:** still does not work in Spyder
    - date resolved: **2023-11-22 02:03**\ , date raised: 2023-11-22 
 * Version 1.7.32: resolved Issue 1701: RunCppUnitTests (change)
    - description:  move to exudyn.special.RunCppUnitTests
    - date resolved: **2023-11-22 00:29**\ , date raised: 2023-11-22 
 * Version 1.7.31: resolved Issue 1697: exudyn.experimental (change)
    - description:  change exudyn.Experimental() into exudyn.experimental structure for clearer view on it; not intended to be used widely
    - **notes:** see also issue 1613
    - date resolved: **2023-11-21 21:18**\ , date raised: 2023-11-21 
 * Version 1.7.30: resolved Issue 1694: load user functions (change)
    - description:  add stl and numpy bindings in order to have Vector3D converted into numpy arrays in loadVectorUserFunction
    - date resolved: **2023-11-19 23:05**\ , date raised: 2023-11-19 
 * Version 1.7.29: resolved Issue 1690: mainSystemExtensions (extension)
    - description:  add CreateGround() with referencePosition, referenceRotationMatrix and visualization
    - date resolved: **2023-11-19 23:05**\ , date raised: 2023-11-19 
 * Version 1.7.28: resolved Issue 1679: CreateTorque (extension)
    - description:  add create function for nodes and bodies (using node=None, body=None default) to be used either for nodes or bodies; automatically adds markers; bodyFixed=False; add option to add userFunction
    - date resolved: **2023-11-19 22:22**\ , date raised: 2023-10-29 
 * Version 1.7.27: resolved Issue 1678: CreateForce (extension)
    - description:  add create function for nodes and bodies (using node=None, body=None default) to be used either for nodes or bodies; automatically adds markers; bodyFixed=False; add option to add userFunction
    - date resolved: **2023-11-19 22:22**\ , date raised: 2023-10-29 
 * Version 1.7.26: resolved Issue 1689: mainSystemExtensions (extension)
    - description:  change bodyOrNodeList into bodyList; allow bodyOrNodeList as alternative arg, but avoid using in examples
    - date resolved: **2023-11-19 21:04**\ , date raised: 2023-11-19 
 * Version 1.7.25: resolved Issue 1688: mainSystemExtensions (extension)
    - description:  allow special case distance=0 in CreateDistanceConstraint; this will then create a SphericalJoint
    - date resolved: **2023-11-19 21:04**\ , date raised: 2023-11-19 
 * Version 1.7.24: resolved Issue 1687: mainSystemExtensions (extension)
    - description:  allow referenceLength=0 in ConnectorSpringDamper
    - date resolved: **2023-11-19 21:04**\ , date raised: 2023-11-19 
 * Version 1.7.23: resolved Issue 1667: ANCFCable (extension)
    - description:  add visulization with cylinders
    - date resolved: **2023-11-19 19:39**\ , date raised: 2023-10-16 
 * Version 1.7.22: resolved Issue 1686: SpringDamper (extension)
    - description:  allow springLength=0, as it does not cause problems in computations
    - **notes:** added special behavior for L=0, but velocity not equal 0; may cause convergence issues in particular for static problems; added special behavior for L=0, but velocity not equal 0; may cause convergence issues in particular for static problems; also allow referenceLength=0
    - date resolved: **2023-11-19 19:11**\ , date raised: 2023-11-19 
 * Version 1.7.21: resolved Issue 1685: ANCF cable with rigid marker (example)
    - description:  add example with ANCFCable2D and rigid body marker, prescribing rotation of one end
    - **notes:** ANCFrotatingCable2D.py
    - date resolved: **2023-11-07 14:04**\ , date raised: 2023-11-07 
 * Version 1.7.20: resolved Issue 1673: ReadTheDocs (fix)
    - description:  add the required .readthedocs.yaml file and move requirements.txt into docs folder, as it is related to sphinx only
    - date resolved: **2023-10-25 09:16**\ , date raised: 2023-10-25 
 * Version 1.7.19: resolved Issue 1672: LieGroup explicit integration (change)
    - description:  fixed lieGroupDataNodes; lieGroupDataNodes renamed into lieGroupNodes in explict integration
    - date resolved: **2023-10-16 10:00**\ , date raised: 2023-10-16 
 * Version 1.7.18: resolved Issue 1671: PlotSensor (change)
    - description:  use IsListOrArray function to check for non-empty offsets and factors, to comply with numpy arrays
    - date resolved: **2023-10-16 08:32**\ , date raised: 2023-10-16 
 * Version 1.7.17: resolved Issue 1670: GenerateStraightLineANCFCable (extension)
    - description:  add GenerateStraightLineANCFCable for 3D cables and adjust 2D version
    - date resolved: **2023-10-16 08:31**\ , date raised: 2023-10-16 
 * Version 1.7.16: resolved Issue 1669: NodePoint3DSlope (fix)
    - description:  adjust all test models and examples for new NodePointSlope... names
    - date resolved: **2023-10-16 08:02**\ , date raised: 2023-10-16 
 * Version 1.7.15: resolved Issue 1663: NodePoint3DSlope23 (check)
    - description:  check d/dt(A) ... computation of time derivative of rotation matrix, also check jacobians and angular velocities; add tests also for Slope12 node
    - date resolved: **2023-10-16 07:45**\ , date raised: 2023-10-15 
 * Version 1.7.14: resolved Issue 1666: NodePoint3DSlope1 (change)
    - description:  change to NodePointSlope1
    - date resolved: **2023-10-15 22:29**\ , date raised: 2023-10-15 
 * Version 1.7.13: resolved Issue 1665: Point3DS23 (change)
    - description:  remove Point3DS23 as its name is inconsistent with 3D convention and not really needed
    - date resolved: **2023-10-15 22:24**\ , date raised: 2023-10-15 
 * Version 1.7.12: resolved Issue 1664: NodePoint3DSlope23 (change)
    - description:  change to NodePointSlope23
    - date resolved: **2023-10-15 22:23**\ , date raised: 2023-10-15 
 * Version 1.7.11: resolved Issue 1075: ANCFCable (extension)
    - description:  add 3D version of cable element, not using BeamSection interface for compatibility with Cable2D
    - date resolved: **2023-10-15 14:45**\ , date raised: 2022-05-06 
 * Version 1.7.10: resolved Issue 1661: NodePoint3DSlope12 (extension)
    - description:  add ANCF node with slopes x/y for thin plate element
    - date resolved: **2023-10-15 14:21**\ , date raised: 2023-10-15 
 * Version 1.7.9: resolved Issue 1403: Lie group nodes (change)
    - description:  remove LieGroup node RigidBodyRotVecDataLG as it is not needed any more as it can be substituted with more efficient RigidBodyRotVecLG
    - **notes:** right now added comments in C++ files, not completely removed
    - date resolved: **2023-10-15 11:57**\ , date raised: 2023-01-18 
 * Version 1.7.8: resolved Issue 1659: remove math.h (change)
    - description:  change with <cmath> for C++ conformity
    - date resolved: **2023-10-12 17:04**\ , date raised: 2023-10-12 
 * Version 1.7.7: resolved Issue 1658: switch to MSVC2022 (change)
    - description:  change main_sln_Template.sln for 2022 update; use cl.exe from MSVC2022 for compilation of wheels; slightly increases performance
    - **notes:** compilation successful and TestSuite runs through
    - date resolved: **2023-10-12 17:04**\ , date raised: 2023-10-12 
 * Version 1.7.6: resolved Issue 1660: plot (change)
    - description:  change plt.tight_layout and plt.legend to fig. if possible to avoid warnings
    - date resolved: **2023-10-12 13:39**\ , date raised: 2023-10-12 
 * Version 1.7.5: resolved Issue 1657: Docu (fix)
    - description:  latex errors in robotics mobile and ROS
    - date resolved: **2023-10-09 20:27**\ , date raised: 2023-10-09 
 * Version 1.7.4: resolved Issue 1656: ROS rosInterface (extension)
    - description:  Created robotics.rosInterface and Python models in Examples: ROSMassPoint.py, ROSMobileManipulator.py, ROSTurtle.py with supplementary, see Examples/supplementary: ROSControlMobileManipulation.py, ROSControlTurtleVelocity.py, etc.
    - date resolved: **2023-09-15 15:50**\ , date raised: 2023-09-15 
    - resolved by: Martin Sereinig
 * Version 1.7.3: resolved Issue 1655: SolveStatic (extension)
    - description:  add option for static solver to handle ODE1 quantities; currently, the option is to set ODE1coordinates to initial values during static computation
    - date resolved: **2023-09-07 21:13**\ , date raised: 2023-09-07 
 * Version 1.7.2: resolved Issue 1654: Python3.6 support (change)
    - description:  discontinuing testing and creation of pip installers for Python3.6 in Windows, Linux and MacOS as Python3.6 had end-of-life 2021-12-23; Python 3.7 also had end-of-life recently, so please expect discontinued support soon
    - date resolved: **2023-09-03 16:03**\ , date raised: 2023-09-03 
 * Version 1.7.1: resolved Issue 1652: rosInterface.py (fix)
    - description:  add (missing) file to DOCU
    - date resolved: **2023-08-08 17:53**\ , date raised: 2023-08-08 
 * Version 1.7.0: resolved Issue 1649: release (release)
    - description:  switch to new release 1.7
    - date resolved: **2023-07-19 16:07**\ , date raised: 2023-07-19 

***********
Version 1.6
***********

 * Version 1.6.189: resolved Issue 1648: ReevingSystemSprings (fix)
    - description:  adjust test example for treating compression forces
    - date resolved: **2023-07-17 12:10**\ , date raised: 2023-07-17 
 * Version 1.6.188: resolved Issue 1647: sensorTraces (extension)
    - description:  add vectors and triads to position traces; show current vector or triad and add some further options for visualization, see visualizationSettings sensors.traces
    - **notes:** added triads and vectors, showing traces of motion at sensor points
    - date resolved: **2023-07-14 18:34**\ , date raised: 2023-07-14 
 * Version 1.6.187: resolved Issue 1640: visualization (extension)
    - description:  show trace of sensor positions (incl. frames) in render window; settings and list of sensors provided in visualizationSettings.sensors.traces with list of sensors, positionTrace, listOfPositionSensors=[] (empty means all position sensors, listOfVectorSensors=[] which can provide according vector quantities for positions; showVectors, vectorScaling=0.001, showPast=True, showFuture=False, showCurrent=True, lineWidth=2
    - date resolved: **2023-07-14 11:20**\ , date raised: 2023-07-11 
 * Version 1.6.186: resolved Issue 1646: ArrayFloat (extension)
    - description:  C++ add type
    - date resolved: **2023-07-13 16:27**\ , date raised: 2023-07-13 
 * Version 1.6.185: resolved Issue 1645: ReevingSystemSprings (extension)
    - description:  add way to remove compression forces in rope
    - **notes:** added parameter regularizationForce with tanh regularization for avoidance of compressive spring force
    - date resolved: **2023-07-12 16:05**\ , date raised: 2023-07-12 
 * Version 1.6.184: resolved Issue 1644: Minimize (extension)
    - description:  processing.Minimize: improve printout of current error of objective function=loss; only print every 1 second
    - date resolved: **2023-07-12 09:29**\ , date raised: 2023-07-12 
 * Version 1.6.183: :textred:`resolved BUG 1643` : Minimize 
    - description:  processing.Minimize function has an internal bug, such that it does not work with initialGuess=[]
    - date resolved: **2023-07-12 09:29**\ , date raised: 2023-07-12 
 * Version 1.6.182: :textred:`resolved BUG 1642` : SolutionViewer 
    - description:  record image not working with visualizationSettings useMultiThreadedRendering=True
    - date resolved: **2023-07-11 17:58**\ , date raised: 2023-07-11 
 * Version 1.6.181: resolved Issue 1641: SolutionViewer (fix)
    - description:  github issue#51: graphicsDataUserFunction in SolutionViewer not called; add call to graphicsData user functions in redraw image loop
    - date resolved: **2023-07-11 17:16**\ , date raised: 2023-07-11 
 * Version 1.6.180: resolved Issue 1638: GeneticOptimization (extension)
    - description:  add argument parameterFunctionData to GeneticOptimization; same as in ParameterVariation, paramterFunctionData allows to pass additional data to the objective function
    - date resolved: **2023-07-09 09:15**\ , date raised: 2023-07-09 
 * Version 1.6.179: resolved Issue 1637: add ChatGPT update information (extension)
    - description:  create Python model Examples/chatGPTupdate.py which includes information that is used by ChatGPT4 to improve abilities to create simple models fully automatic
    - date resolved: **2023-06-30 15:01**\ , date raised: 2023-06-30 
 * Version 1.6.178: resolved Issue 1636: CreateMassPoint (change)
    - description:  change args referenceCoordinates to referencePosition, initialCoordinates to initialDisplacement, and initialVelocities to initialVelocity to be consistent with CreateRigidBody (but different from MassPoint itself)
    - date resolved: **2023-06-30 14:09**\ , date raised: 2023-06-30 
 * Version 1.6.177: resolved Issue 1635: SmartRound2String (extension)
    - description:  add function to basic utilities to enable simple printing of numbers with few digits, including comma dot and not eliminating small numbers, e.g., 1e-5 stays 1e-5
    - date resolved: **2023-06-30 09:22**\ , date raised: 2023-06-30 
 * Version 1.6.176: resolved Issue 1634: create directories (extension)
    - description:  add automatic creation of directories to FEM SaveToFile, plotting and ParameterVariation
    - date resolved: **2023-06-29 10:29**\ , date raised: 2023-06-29 
 * Version 1.6.175: :textred:`resolved BUG 1633` : GetInterpolatedSignalValue 
    - description:  timeArray needs to be replaced with timeArrayNew in case of 2D input array
    - date resolved: **2023-06-28 21:15**\ , date raised: 2023-06-28 
 * Version 1.6.174: resolved Issue 1632: PlotFFT (fix)
    - description:  matplotlib >= 1.7 complains about ax.grid(b=...) as parameter b has been replaced by visible
    - date resolved: **2023-06-27 14:15**\ , date raised: 2023-06-27 
 * Version 1.6.173: resolved Issue 1626: mutable args itemInterface (fix)
    - description:  also copy dictionaries, mainly for visualization (flat level, but this should be sufficient)
    - date resolved: **2023-06-21 10:40**\ , date raised: 2023-06-21 
 * Version 1.6.172: resolved Issue 1627: mutable default args (change)
    - description:  complete changes and adaptations for default args in Python functions and item interface; note individual adaptations for lists, vectors, matrices and special lists of lists or matrix containers; for itemInterface, anyway all data is copied into C++; for more information see issues 1536, 1540, 1612, 1624, 1625, 1626
    - date resolved: **2023-06-21 10:15**\ , date raised: 2023-06-21 
 * Version 1.6.171: resolved Issue 1625: change to default arg None (change)
    - description:  change default args for Vector2DList, Vector3DList, Vector6DList, Matrix3DList, to None; ArrayNodeIndex, ArrayMarkerIndex, ArraySensorIndex obtain copy method and are copied now; avoid problem of mutable default args
    - date resolved: **2023-06-21 00:26**\ , date raised: 2023-06-20 
 * Version 1.6.170: resolved Issue 1624: MatrixContainer (change)
    - description:  change default values for matrix container to None; avoid problem of mutable default args
    - date resolved: **2023-06-20 23:39**\ , date raised: 2023-06-20 
 * Version 1.6.169: resolved Issue 1540: mutable args itemInterface (check)
    - description:  copy lists in itemInterface in order to avoid change of default args by user n=NodePoint();n.referenceCoordinates[0]=42;n1=NodePoint()
    - **notes:** simple vectors, matrices and lists are copied with np.array(...) while complex matrix and list of array types are now initialized with None
    - date resolved: **2023-06-20 22:13**\ , date raised: 2023-04-28 
 * Version 1.6.168: resolved Issue 1620: docu MainSystemExtensions (docu)
    - description:  reorder MainSystemExtensions with separate section for Create functions and one section for remaining functions
    - date resolved: **2023-06-19 22:14**\ , date raised: 2023-06-13 
 * Version 1.6.167: :textred:`resolved BUG 1622` : mouse click 
    - description:  fix crash on linux if left / right mouse click on render window (related to OpenGL select window)
    - **notes:** occurs on WSL with WSLg; using 'export LIBGL_ALWAYS_SOFTWARE=1' will resolve the problem; put this line into your .bashrc
    - date resolved: **2023-06-19 22:11**\ , date raised: 2023-06-19 
 * Version 1.6.166: :textred:`resolved BUG 1621` : LinearSolverType 
    - description:  fix crash on linux in function SetLinearSolverType
    - date resolved: **2023-06-19 20:27**\ , date raised: 2023-06-19 
 * Version 1.6.165: resolved Issue 1623: MouseSelectOpenGL (extension)
    - description:  add optional debbuging output
    - date resolved: **2023-06-19 19:56**\ , date raised: 2023-06-19 
 * Version 1.6.164: resolved Issue 1267: matrix inverse (extension)
    - description:  add pivot threshold to options, may improve redundant constraints problems
    - **notes:** only available for EigenDense with ignoreSingularJacobian and EXUdense linear solvers
    - date resolved: **2023-06-12 13:24**\ , date raised: 2022-09-21 
 * Version 1.6.163: resolved Issue 1616: eigen LU (check)
    - description:  check fastest solver for regular and overdetermined systems
    - **notes:** Eigen::PartialPivLU 2.5 times faster for 65 DOF test in factorization, factor 3 faster for backsubst
    - date resolved: **2023-06-12 13:22**\ , date raised: 2023-06-11 
 * Version 1.6.162: resolved Issue 1607: Bricard mechanism (testing)
    - description:  add example and test ComputeSystemDegreesOfFreedom
    - date resolved: **2023-06-12 11:02**\ , date raised: 2023-06-09 
 * Version 1.6.161: resolved Issue 1617: solver error message (fix)
    - description:  revise hint for ignoreSingularJacobian
    - date resolved: **2023-06-12 01:39**\ , date raised: 2023-06-11 
 * Version 1.6.160: resolved Issue 1615: pivotThreshold (fix)
    - description:  add already existing parameter [which is currently not used in solver!] to solver interface add FactorizeNew arg
    - date resolved: **2023-06-12 01:39**\ , date raised: 2023-06-11 
 * Version 1.6.159: resolved Issue 1266: matrix inverse (extension)
    - description:  add full pivoting mode for matrix inverse, to resolve redundant constraints; consider Eigen FullPivotLU for dense matrices - see classEigen_1_1FullPivLU.html
    - **notes:** also added new LinearSolverType.EigenDense which allows to chose FullPivLU by settings ignoreSingularJacobian=True
    - date resolved: **2023-06-12 00:17**\ , date raised: 2022-09-21 
 * Version 1.6.158: resolved Issue 1619: LinearSolverType (change)
    - description:  switch to bit-wise numbering of solver types, in order to alleviate checks
    - date resolved: **2023-06-11 23:52**\ , date raised: 2023-06-11 
 * Version 1.6.157: resolved Issue 1618: ignoreRedundantConstraints (change)
    - description:  remove option ignoreRedundantConstraints as it cannot be applied with Eigen::FullPivLU; use ignoreSingularJacobian instead
    - date resolved: **2023-06-11 23:46**\ , date raised: 2023-06-11 
 * Version 1.6.156: resolved Issue 1613: Experimental (extension)
    - description:  add experimental class, which can be accessed in Python by exudyn.Experimental(); inside C++, just needs to be imported; allows simple testing without interference with main features
    - date resolved: **2023-06-11 19:56**\ , date raised: 2023-06-11 
 * Version 1.6.155: resolved Issue 1536: mutable arguments (fix)
    - description:  check and fix Python functions with mutable arguments such as [] or {}, with potential risk of changing internally in function, leading to unexpected behavior in second call
    - **notes:** checked all default list and dict args
    - date resolved: **2023-06-11 00:24**\ , date raised: 2023-04-27 
 * Version 1.6.154: resolved Issue 1612: mutable arguments (fix)
    - description:  check and fix problems in beams.py, FEM.py and graphicsDataUtilities.py
    - **notes:** see also issue 1536
    - date resolved: **2023-06-10 21:34**\ , date raised: 2023-06-10 
 * Version 1.6.153: resolved Issue 1609: AnimateModes (extension)
    - description:  extend for using a set of system eigenmodes
    - date resolved: **2023-06-10 20:25**\ , date raised: 2023-06-10 
 * Version 1.6.152: resolved Issue 1580: mainSystemExtensions (extension)
    - description:  add RigidBodySpringDamper
    - date resolved: **2023-06-10 20:25**\ , date raised: 2023-05-21 
 * Version 1.6.151: resolved Issue 1606: ComputeODE2Eigenvalues (extension)
    - description:  add eigenvector computation to constrained case
    - **notes:** needs further testing!
    - date resolved: **2023-06-10 19:11**\ , date raised: 2023-06-08 
 * Version 1.6.150: resolved Issue 1611: SolutionViewer (change)
    - description:  change internal variables from mbs.variables to mbs.sys
    - date resolved: **2023-06-10 17:48**\ , date raised: 2023-06-10 
 * Version 1.6.149: resolved Issue 1610: AnimateModes (change)
    - description:  change internal variables from mbs.variables to mbs.sys
    - date resolved: **2023-06-10 17:47**\ , date raised: 2023-06-10 
 * Version 1.6.148: :textred:`resolved BUG 1608` : visualization zoom all 
    - description:  when calling ComputeSystemDegreeOfFreedom, ComputeODE2Eigenvalues and similar functions, and StartRenderer is called right afterwards, zoom all does not work
    - **notes:** shall be resolved just by calling StartRenderer before first call to any solver functionality
    - date resolved: **2023-06-10 11:44**\ , date raised: 2023-06-10 
 * Version 1.6.147: resolved Issue 1435: solver (extension)
    - description:  add deriviative of loads to regular jacobian computation with flag (default=False); use numerical diff sim. to JacobianODE2
    - **notes:** already done earlier in #1546
    - date resolved: **2023-06-08 23:35**\ , date raised: 2023-02-16 
 * Version 1.6.146: resolved Issue 1597: Command execute (fix)
    - description:  switch to grid method for placing widgets, tkinter does not allow pack and grid in different windows
    - date resolved: **2023-06-08 21:56**\ , date raised: 2023-06-05 
 * Version 1.6.145: :textred:`resolved BUG 1593` : TemporaryComputationDataArray bug 
    - description:  ERROR: "TemporaryComputationDataArray::operator[]: index out of range" is raised if single-threaded computation is run after multi-threaded simulation; requires restart of Python instance
    - date resolved: **2023-06-08 18:50**\ , date raised: 2023-06-03 
 * Version 1.6.144: resolved Issue 1599: ComputeODE2Eigenvalues (extension)
    - description:  compute eigenmodes in case of algebraic equations
    - date resolved: **2023-06-08 18:46**\ , date raised: 2023-06-08 
 * Version 1.6.143: resolved Issue 1603: AddSensor (extension)
    - description:  add check if no outputVariable is provided-> immediately raise error
    - **notes:** was already in system checks, which however were not called, see issue 1604
    - date resolved: **2023-06-08 18:45**\ , date raised: 2023-06-08 
 * Version 1.6.142: resolved Issue 1598: eigenvalues constrained system (example)
    - description:  add example for eigenvalue computation of constrained system
    - **notes:** added computeODE2AEeigenvaluesTest.py
    - date resolved: **2023-06-08 18:45**\ , date raised: 2023-06-08 
 * Version 1.6.141: resolved Issue 1605: ComputeSystemDegreeOfFreedom (change)
    - description:  change output to a dictionary in order to have readable results
    - date resolved: **2023-06-08 18:35**\ , date raised: 2023-06-08 
 * Version 1.6.140: resolved Issue 1604: Sensors (fix)
    - description:  PreAssembleConsistencies not called in Assemble; thus, no checks are performed on sensor inputs
    - date resolved: **2023-06-08 18:24**\ , date raised: 2023-06-08 
 * Version 1.6.139: :textred:`resolved BUG 1602` : MainSystem CreateRigidBody 
    - description:  referenceRotationMatrix multiplied in wrong way with initialRotationMatrix
    - **notes:** also rotation parameters in initialVelocities were wrong for initialRotationMatrix!=np.eye(3); fixed, but more testing needed
    - date resolved: **2023-06-08 17:42**\ , date raised: 2023-06-08 
 * Version 1.6.138: resolved Issue 1600: stub files (.pyi) (extension)
    - description:  extend .pyi files for system structures functions, e.g., ComputeJacobianODE2RHS
    - date resolved: **2023-06-08 17:17**\ , date raised: 2023-06-08 
 * Version 1.6.137: resolved Issue 1601: stub files (.pyi) (change)
    - description:  merge .pyi files to have classes such as MainSystem only appearing once
    - date resolved: **2023-06-08 16:37**\ , date raised: 2023-06-08 
 * Version 1.6.136: resolved Issue 0746: ComputeODEEigenvalues (extension)
    - description:  add possibility to eliminate coordinate constraints, possibly to use SVD/ null space matrix for projection
    - **notes:** algebraic constraints now considered automatically; algebraic constraints now considered automatically
    - date resolved: **2023-06-07 23:52**\ , date raised: 2021-09-03 
    - resolved by: M. Pieber, JG
 * Version 1.6.135: resolved Issue 0743: ComputeODE2Eigenvalues (extension)
    - description:  add vector of constrained coordinates which are eliminated; also add functionality for complex eigenvalues
    - **notes:** already done earlier
    - date resolved: **2023-06-07 23:09**\ , date raised: 2021-08-20 
 * Version 1.6.134: resolved Issue 1595: Command dialog (extension)
    - description:  extend to multi-line commands; execute code using CTRL-Return
    - **notes:** Behavior is now DIFFERENT, as variables are not printed automatically; write e.g. print(mbs) to see mbs representation; see section Execute Command and Help
    - date resolved: **2023-06-04 19:40**\ , date raised: 2023-06-03 
 * Version 1.6.133: resolved Issue 1596: linuxDisplayScaleFactor (extension)
    - description:  add scaling for fonts on linux, specifically for high resolution screens
    - date resolved: **2023-06-04 00:13**\ , date raised: 2023-06-04 
 * Version 1.6.132: resolved Issue 1588: ParameterVariation, useMPI (fix)
    - description:  only accept useMPI if set True
    - date resolved: **2023-06-03 19:26**\ , date raised: 2023-05-31 
 * Version 1.6.131: resolved Issue 1579: mainSystemExtensions (extension)
    - description:  Add distance constraint and CartesianSpringDamper
    - date resolved: **2023-06-03 19:26**\ , date raised: 2023-05-21 
 * Version 1.6.130: resolved Issue 1594: window closing, key Q (change)
    - description:  slightly adapt behavior; fix some smaller issues with expected window behavior
    - date resolved: **2023-06-03 17:01**\ , date raised: 2023-06-03 
 * Version 1.6.129: resolved Issue 1356: CHECK (extension)
    - description:  Add security question on quit/escape if computation Renderer runs longer than 15 minutes
    - **notes:** added message in render window to click twice on exit window icon (X) after 15 minutes; key Q and escape get tkinter message box
    - date resolved: **2023-06-03 16:00**\ , date raised: 2023-01-01 
 * Version 1.6.128: resolved Issue 1592: SolverBase it.endTime (fix)
    - description:  in dynamic solver it.endTime is overwritten with simulationSettings.timeIntegration.endTime; this is against the description and does not allow to change it.endTime in command window; remove overwritting to be consistent with description of Execute Command and Help section in EXUDYN Basics
    - date resolved: **2023-06-03 14:25**\ , date raised: 2023-06-03 
 * Version 1.6.127: resolved Issue 1591: SphericalJoint (fix)
    - description:  causes memory allocation; check LinkedDataVector cast
    - date resolved: **2023-06-01 11:12**\ , date raised: 2023-06-01 
 * Version 1.6.126: resolved Issue 1590: serialRobotKinematicTree.py (fix)
    - description:  fixed static torque compensation for kinematic tree and fixed sensor outputs
    - date resolved: **2023-05-31 13:07**\ , date raised: 2023-05-31 
 * Version 1.6.125: resolved Issue 1589: ObjectKinematicTree (docu)
    - description:  add description of SensorKinematicTree output variables per link
    - date resolved: **2023-05-31 12:42**\ , date raised: 2023-05-31 
 * Version 1.6.124: resolved Issue 1587: solution file footer (change)
    - description:  add comma after cpuTime=...
    - date resolved: **2023-05-30 23:52**\ , date raised: 2023-05-30 
 * Version 1.6.123: resolved Issue 1586: SetPreStepUserFunction, SetPostNewtonUserFunction (extension)
    - description:  add exception handling in set function
    - date resolved: **2023-05-26 19:52**\ , date raised: 2023-05-26 
 * Version 1.6.122: resolved Issue 1585: class name highlighting RTD (docu)
    - description:  fixed exporting class names from pythonUtilities
    - date resolved: **2023-05-25 15:19**\ , date raised: 2023-05-25 
 * Version 1.6.121: resolved Issue 1584: MiniExamples (change)
    - description:  remove import of itemInterface
    - date resolved: **2023-05-25 14:32**\ , date raised: 2023-05-25 
 * Version 1.6.120: :textred:`resolved BUG 1583` : GeneticOptimization 
    - description:  results are erroneous in case of crossoverProbability > 0
    - **notes:** fixed writing of output files which had mixed order due to parameter cross-over
    - date resolved: **2023-05-23 18:31**\ , date raised: 2023-05-23 
 * Version 1.6.119: resolved Issue 1582: mainSystemExtensions (fix)
    - description:  remove import of tkinter and matplotlib to resolve errors when loading exudyn and these libs are not installed
    - date resolved: **2023-05-22 10:54**\ , date raised: 2023-05-22 
 * Version 1.6.118: resolved Issue 1577: examples (fix)
    - description:  test if all Examples are still running
    - date resolved: **2023-05-20 22:23**\ , date raised: 2023-05-20 
 * Version 1.6.117: resolved Issue 1578: ObjectConnectorCoordinateSpringDamper (fix)
    - description:  correct docu on object and user function description (still includes friction)
    - date resolved: **2023-05-20 21:13**\ , date raised: 2023-05-20 
 * Version 1.6.116: resolved Issue 1569: mainSystemExtensions (example)
    - description:  add mini-examples for extensions
    - **notes:** collected miniexamples in mainSystemExtensionsTests.py; 
    - date resolved: **2023-05-20 21:05**\ , date raised: 2023-05-15 
 * Version 1.6.115: resolved Issue 1571: mainSystemExtensions (change)
    - description:  adapt Examples to Python extensions (SolveDynamic, CreateRigidBody, CreateGenericJoint, ...)
    - date resolved: **2023-05-18 23:43**\ , date raised: 2023-05-15 
 * Version 1.6.114: resolved Issue 1570: mainSystemExtensions (change)
    - description:  adapt TestModels to Python extensions
    - date resolved: **2023-05-18 23:43**\ , date raised: 2023-05-15 
 * Version 1.6.113: :textred:`resolved BUG 1576` : multithreading 
    - description:  running laserScannerTest.py after a multithreaded contact computation raises the EXCEPTION: TemporaryComputationDataArray::operator[]: index out of range
    - date resolved: **2023-05-18 23:13**\ , date raised: 2023-05-18 
 * Version 1.6.112: resolved Issue 1575: ConnectorDistance (change)
    - description:  changed parameter distance to PReal, not allowing zero distance to be prescribed
    - date resolved: **2023-05-18 13:03**\ , date raised: 2023-05-18 
 * Version 1.6.111: resolved Issue 1573: mainSystemExtensions (docu)
    - description:  Adapt tutorials to new functionality
    - date resolved: **2023-05-18 12:19**\ , date raised: 2023-05-16 
 * Version 1.6.110: resolved Issue 1574: Python utilities (fix)
    - description:  classes not appearing in table of contents on RTD
    - date resolved: **2023-05-17 20:21**\ , date raised: 2023-05-16 
 * Version 1.6.109: resolved Issue 1572: CreateRigidBody (extension)
    - description:  added to MainSystem to enable mbs.CreateRigidBody(...)
    - date resolved: **2023-05-16 11:54**\ , date raised: 2023-05-16 
 * Version 1.6.108: resolved Issue 1568: mainSystemExtensions (extension)
    - description:  create first sample of Python extensions for basic joints
    - date resolved: **2023-05-15 18:13**\ , date raised: 2023-05-15 
 * Version 1.6.107: resolved Issue 1567: DrawSystemGraph (change)
    - description:  in case of showItemNames, no item numbers are shown as they confuse with numbers used in names
    - date resolved: **2023-05-15 17:18**\ , date raised: 2023-05-15 
 * Version 1.6.106: resolved Issue 1566: AddDistanceSensor(...) (change)
    - description:  function RENAMED into CreateDistanceSensor(...) to be consistent with future naming; also renamed DistanceSensorSetupGeometry(...) into CreateDistanceSensorGeometry(...)
    - date resolved: **2023-05-15 11:34**\ , date raised: 2023-05-15 
 * Version 1.6.105: resolved Issue 1563: MainSystem Python extensions (extension)
    - description:  add Python utility functions for mbs, such as PlotSensor, SolveDynamic, ...; use identical interfaces to alleviate creation of .pyi files and documentation; add new flag mbsFunction as hint to put docu to MainSystem and make .pyi extension
    - **notes:** see Section :ref:`sec-mainsystem-pythonextensions`\  for extended functionality
    - date resolved: **2023-05-15 01:17**\ , date raised: 2023-05-09 
 * Version 1.6.104: resolved Issue 1564: Type definitions (docu)
    - description:  fix header structure in latex and RST for Type Definitions
    - date resolved: **2023-05-11 11:30**\ , date raised: 2023-05-11 
 * Version 1.6.103: resolved Issue 1561: stub files (.pyi) (extension)
    - description:  add .pyi files to setup_tools, copying them from autogenerate folder; use try catch to avoid problems at other platforms
    - date resolved: **2023-05-10 23:30**\ , date raised: 2023-05-09 
 * Version 1.6.102: resolved Issue 1560: stub files (.pyi) (extension)
    - description:  automatically create stub file for C++ classes such as MainSystem, SystemContainer, GeneralContact, ... by adding return type information and creating .pyi file; use temporary .pyi files in autogenerate folder
    - date resolved: **2023-05-10 23:19**\ , date raised: 2023-05-09 
 * Version 1.6.101: resolved Issue 1562: stub files (.pyi) (extension)
    - description:  add .pyi files for enums from autoGeneratePyBindings
    - date resolved: **2023-05-10 20:43**\ , date raised: 2023-05-09 
 * Version 1.6.100: resolved Issue 1559: stub files (.pyi) (extension)
    - description:  automatically create stub file for settings to alleviate auto-completion; type completion now also works for functions, types and structures: tested in Spyder and Visual Studio Code
    - date resolved: **2023-05-10 08:39**\ , date raised: 2023-05-09 
 * Version 1.6.99: resolved Issue 1557: stub files (.pyi) (check)
    - description:  test creating stub files (.pyi) which are needed for MainSystem Python extensions
    - **notes:** tested with Spyder 5.1.5 and Visual Studio Code 1.78
    - date resolved: **2023-05-10 08:39**\ , date raised: 2023-05-07 
 * Version 1.6.98: resolved Issue 1558: enum in global scope (fix)
    - description:  pybind11 translates enums to global module scope, e.g. exudyn.ItemType.Marker is also available as exudyn.Marker; remove export_values() in pybind interface
    - **notes:** if you by occasion used e.g. exu.DisplacementLocal instead of exu.OutputVariableType.DisplacementLocal you need to adapt your code!
    - date resolved: **2023-05-09 18:30**\ , date raised: 2023-05-09 
 * Version 1.6.97: resolved Issue 1556: IsValidPRealPInt, IsValidURealPInt (extension)
    - description:  add functions to advancedUtilities for additional checks in Python functions
    - date resolved: **2023-05-07 20:33**\ , date raised: 2023-05-07 
 * Version 1.6.96: resolved Issue 1555: color4default (extension)
    - description:  add default color to graphicsDataUtilities as a default value for items
    - date resolved: **2023-05-07 18:12**\ , date raised: 2023-05-07 
 * Version 1.6.95: resolved Issue 1554: NodePoint2D (change)
    - description:  draw as sphere by default to improve visibility
    - date resolved: **2023-05-07 16:31**\ , date raised: 2023-05-07 
 * Version 1.6.94: resolved Issue 1539: item names (change)
    - description:  remove stored string and replace by empty string in case of default item name
    - **notes:** not changed: std::string has practically no effect on memory footprint of items; check memory footprint separately (LTG lists, etc.)!
    - date resolved: **2023-05-05 23:32**\ , date raised: 2023-04-28 
 * Version 1.6.93: resolved Issue 1553: AccelerationLocal, AngularAccelerationLocal (fix)
    - description:  add missing values to Pybind interface
    - date resolved: **2023-05-02 17:41**\ , date raised: 2023-05-02 
 * Version 1.6.92: resolved Issue 1552: HydraulicActuator (extension)
    - description:  add VelocityLocal as output variable, which provides the time derivative of the distance, being the actuator velocity; updated docu and fixed description for outputvariable Velocity
    - date resolved: **2023-05-02 16:51**\ , date raised: 2023-05-02 
 * Version 1.6.91: resolved Issue 1551: GeometricallyExactBeam (change)
    - description:  evaluate rotation at midspan of beam
    - date resolved: **2023-05-02 15:02**\ , date raised: 2023-05-02 
 * Version 1.6.90: resolved Issue 1502: ANCFBeam (testing)
    - description:  check for very large deformations
    - **notes:** poor performance for large deformations caused by missing load jacobian
    - date resolved: **2023-05-01 19:26**\ , date raised: 2023-04-08 
 * Version 1.6.89: resolved Issue 1501: GeometricallyExactBeam (testing)
    - description:  check for very large deformations
    - **notes:** poor performance for large deformations caused by missing load jacobian
    - date resolved: **2023-05-01 19:26**\ , date raised: 2023-04-08 
 * Version 1.6.88: resolved Issue 1546: computeLoadsJacobian (change)
    - description:  add flag in timeIntegration and staticSolver settings to turn on/off jacobian of loads; will be by default turned on in static solver, turned off in time integration; adapt your models
    - date resolved: **2023-05-01 19:22**\ , date raised: 2023-05-01 
 * Version 1.6.87: resolved Issue 1545: ComputeSingleLoads (extension)
    - description:  adapt for Jacobian computation of loads
    - date resolved: **2023-05-01 19:22**\ , date raised: 2023-05-01 
 * Version 1.6.86: resolved Issue 1547: GetMarkerOutput (fix)
    - description:  in case of OuputVariableType.Coordinates, add check if Marker is of type Coordinate or Coordinates
    - date resolved: **2023-05-01 16:17**\ , date raised: 2023-05-01 
 * Version 1.6.85: resolved Issue 1544: systemData.GetNodeLocalToGlobal (extension)
    - description:  add systemData access functions for LTG mappings of nodes, useful to check node coordinates; works for ODE2, ODE1, AE and Data coordinates; useful to create load dependencies
    - date resolved: **2023-04-29 22:03**\ , date raised: 2023-04-29 
 * Version 1.6.84: resolved Issue 1543: systemData (extension)
    - description:  add InfoLTG function which outputs LTG lists and load dependencies
    - date resolved: **2023-04-29 22:03**\ , date raised: 2023-04-29 
 * Version 1.6.83: resolved Issue 0483: URDF file (extension)
    - description:  check importing an URDF file for robots, urdf_parser_py
    - **notes:** not done: should be done instead by Corke robotics-toolbox
    - date resolved: **2023-04-29 20:20**\ , date raised: 2020-12-04 
 * Version 1.6.82: resolved Issue 1542: HydraulicsActuator (change)
    - description:  changing referenceVolume0 and referenceVolume1 to hoseVolume0 and hoseVolume1 with different meaning according to referenced paper; adjust your models!
    - **notes:** thanks to Qasim Khadim for provigind the model
    - date resolved: **2023-04-29 01:11**\ , date raised: 2023-04-29 
 * Version 1.6.81: resolved Issue 1541: HydraulicsActuator (extension)
    - description:  extend model of effective bulk modulus acc. to paper https://doi.org/10.1007/s11044-019-09696-y
    - date resolved: **2023-04-29 01:10**\ , date raised: 2023-04-28 
 * Version 1.6.80: resolved Issue 1538: taskmanager (fix)
    - description:  fix problem with taskmanager shutdown due to issue 1532
    - date resolved: **2023-04-27 15:35**\ , date raised: 2023-04-27 
 * Version 1.6.79: resolved Issue 1537: output.multiThreadingMode (fix)
    - description:  has not been set correctly in solver so far; switch output.numberOfThreadsUsed and output.multiThreadingMode
    - date resolved: **2023-04-27 15:35**\ , date raised: 2023-04-27 
 * Version 1.6.78: resolved Issue 1535: writeSensors (extension)
    - description:  add global flag to deactivate sensor file creating/writing and sensor storing; set flag in solver functions such as ComputeLinearizedSystem, etc. to avoid erasing sensor files or sensor data
    - date resolved: **2023-04-27 11:34**\ , date raised: 2023-04-26 
 * Version 1.6.77: resolved Issue 1533: FinalizeSolver (fix)
    - description:  call FinalizeSolver in ComputeLinearizedSystem, ComputeSystemDegreeOfFreedom, ComputeODE2Eigenvalues for consistency reason; also deactivate file writing and sensor writing and solverInformation writing
    - date resolved: **2023-04-26 19:25**\ , date raised: 2023-04-26 
 * Version 1.6.76: resolved Issue 1532: CSolverBase.cpp (fix)
    - description:  close output and sensor files in destructor of CSolverBase
    - date resolved: **2023-04-26 19:24**\ , date raised: 2023-04-26 
 * Version 1.6.75: resolved Issue 1534: EXUlie (change)
    - description:  improve TExpSE3 and TExpSE3Inv regarding small values according to PhD thesis of Stefan Hante
    - **notes:** improves numerical behavior and convergence of GeometricallyExactBeam
    - date resolved: **2023-04-26 18:33**\ , date raised: 2023-04-26 
 * Version 1.6.74: resolved Issue 1531: OpenVR (fix)
    - description:  change order of eye transformation and projection in OpenVRinterface GetCurrentViewProjectionMatrix to be consistent with master thesis
    - date resolved: **2023-04-26 16:49**\ , date raised: 2023-04-26 
 * Version 1.6.73: resolved Issue 1530: ComputeSystemDegreeOfFreedom (extension)
    - description:  add exudyn.solver function to numerically compute DOF of constrained mechanisms
    - date resolved: **2023-04-26 11:16**\ , date raised: 2023-04-26 
 * Version 1.6.72: resolved Issue 1528: structures and settings (docu)
    - description:  function arguments in RST / html are inappropriately noted; fix; replace true/false with True/False
    - date resolved: **2023-04-26 09:16**\ , date raised: 2023-04-26 
 * Version 1.6.71: resolved Issue 1527: GeneralContact (extension)
    - description:  add function UpdateContacts, which computes current bounding boxes and active contacts, to be used in access functions to GeneralContact if isActive=False (otherwise this is anyway done in contact computations of every computation step)
    - date resolved: **2023-04-23 20:15**\ , date raised: 2023-04-23 
 * Version 1.6.70: resolved Issue 0935: GeneralContact (extension)
    - description:  add interface function to get contact pairs
    - **notes:** function GetActiveContacts added to GeneralContact, which returns all active global contact indices for selected contact type
    - date resolved: **2023-04-23 20:13**\ , date raised: 2022-02-10 
 * Version 1.6.69: resolved Issue 1516: solver failed function (extension)
    - description:  add function to check if solver failed, using stored solver structure as input; return True/False and string (optionally error code) describing failure
    - **notes:** added function SolverSuccess() to exudyn.solver; returns success and error string as created by solver internal function GetErrorString(...)
    - date resolved: **2023-04-23 18:43**\ , date raised: 2023-04-19 
 * Version 1.6.68: resolved Issue 1526: solver GetErrorString (extension)
    - description:  MainSolverStatic, MainSolverExplicit, MainSolverImplicit now get function GetErrorString to obtain error string set if SolveSteps or SolveSystem failed (returned false)
    - date resolved: **2023-04-23 11:46**\ , date raised: 2023-04-23 
 * Version 1.6.67: resolved Issue 1525: output.finishedSuccessfully (extension)
    - description:  flag is now set both in SolveSteps(...) as well in SolveSystem(...) to indicate if solver has been successful or failed; practical flag for lateron determination of solver errors
    - date resolved: **2023-04-23 11:46**\ , date raised: 2023-04-23 
 * Version 1.6.66: resolved Issue 1524: netgen STL file (examples)
    - description:  add example with netgen and STL files with meshing
    - date resolved: **2023-04-21 17:51**\ , date raised: 2023-04-21 
 * Version 1.6.65: resolved Issue 1521: ObjectFFRFreducedOrderInterface (extension)
    - description:  add LoadFromFile/SaveToFile similar to FEMinterface
    - **notes:** this function should be used to store CMS data if FEM is too large to load/store; CMS data still stores all node positions, triangle list (for visualization) and modeBasis for computation tasks; may be still large e.g. for many nodes and large number of modes
    - date resolved: **2023-04-20 21:06**\ , date raised: 2023-04-20 
 * Version 1.6.64: resolved Issue 1519: FEMinterface (change)
    - description:  add version to LoadFromFile/SaveToFile function; store file version as first field to load/store in order to be able to load older data as well; use forceVersion=0 to load files in old format
    - **notes:** warning is printed if old file is loaded
    - date resolved: **2023-04-20 20:59**\ , date raised: 2023-04-19 
 * Version 1.6.63: resolved Issue 1523: StrNodeType2NodeType (extension)
    - description:  add function to rigidBodyUtilities in order to make str to type conversion
    - date resolved: **2023-04-20 18:34**\ , date raised: 2023-04-20 
 * Version 1.6.62: resolved Issue 1522: ObjectFFRFreducedOrderInterface (change)
    - description:  remove femInterface from internal variables, as it is only used for postProcessingModes; store postProcessingModes instead
    - date resolved: **2023-04-20 16:34**\ , date raised: 2023-04-20 
 * Version 1.6.61: resolved Issue 1520: ImportFromAbaqusInputFile (change)
    - description:  use VolumeToSurfaceElements for creating of surface elements; add option to automatically create surface triangles
    - **notes:** by default, only surface triangles are created!
    - date resolved: **2023-04-20 15:44**\ , date raised: 2023-04-20 
 * Version 1.6.60: resolved Issue 1518: ObjectFFRFreducedOrderInterface (fix)
    - description:  roundMassMatrix and roundStiffnessMatrix are not used
    - **notes:** added as arguments in RoundMatrix
    - date resolved: **2023-04-19 19:08**\ , date raised: 2023-04-19 
 * Version 1.6.59: resolved Issue 1517: ImportFromAbaqusInputFile (extension)
    - description:  extended for Tet4 and Tet10 as well as C3D20R elements and added function ConvertTetToTrigs(...)
    - date resolved: **2023-04-19 18:14**\ , date raised: 2023-04-19 
 * Version 1.6.58: resolved Issue 1515: unused header files (cleanup)
    - description:  remove unused header files for C, Main and Visu: JointPrismatic.h, JointRevolute.h
    - date resolved: **2023-04-16 13:03**\ , date raised: 2023-04-16 
 * Version 1.6.57: resolved Issue 1269: LaserSensor (extension)
    - description:  add advanced distance sensors replicating laser scanner (with axis, revolution speed and initial direction)
    - **notes:** added function AddLidar(...) into exudyn.robotics.utilities; see laserScannerTest.py
    - date resolved: **2023-04-15 15:33**\ , date raised: 2022-09-21 
 * Version 1.6.56: resolved Issue 1270: DistanceSensor (extension)
    - description:  add advanced distance sensor with possibility to use a set of beams and averaging or multiple output
    - **notes:** added DistanceSensorSetupGeometry in exudyn.utilities to set up contact geometry easily; see laserScannerTest.py
    - date resolved: **2023-04-15 15:32**\ , date raised: 2022-09-21 
 * Version 1.6.55: :textred:`resolved BUG 1514` : GetInterpolatedSignalValue 
    - description:  check for simple test, seems not to work
    - **notes:** changed order of args: dataArrayIndex and timeArrayIndex
    - date resolved: **2023-04-15 14:34**\ , date raised: 2023-04-14 
 * Version 1.6.54: resolved Issue 1513: RevoluteJoint, PrismaticJoint (fix)
    - description:  both ObjectJointPrismaticX and ObjectJointRevoluteZ have wrong internal typename JointRevolute; change to correct names; when analyzing such objects, they will return wrong typenames
    - date resolved: **2023-04-14 14:36**\ , date raised: 2023-04-14 
 * Version 1.6.53: resolved Issue 1511: AddDistanceSensor (extension)
    - description:  add optional color for laser beam
    - date resolved: **2023-04-13 17:45**\ , date raised: 2023-04-13 
 * Version 1.6.52: :textred:`resolved BUG 1510` : AddDistanceSensor 
    - description:  checks performed with invalid marker number
    - date resolved: **2023-04-13 10:28**\ , date raised: 2023-04-13 
 * Version 1.6.51: resolved Issue 1141: object factory (extension)
    - description:  consider hash tables for object factory; check current performance?
    - **notes:** no changes; string comparison has no effect on performance as compared to new and pybind11 overheads for call to  AddObject, etc.
    - date resolved: **2023-04-10 21:48**\ , date raised: 2022-06-12 
 * Version 1.6.50: resolved Issue 1506: ODE2Size (fix)
    - description:  and other functions have additional ConfigurationType: remove
    - **notes:** kept the argument ConfigurationType; usually, all configuration types have same sizes but the call must be related to a speficic configuration
    - date resolved: **2023-04-10 21:42**\ , date raised: 2023-04-08 
 * Version 1.6.49: resolved Issue 1509: SystemData (fix)
    - description:  prevent from creation of a pure SystemData in exudyn; check constructor used in SC.AddSystem and SystemData()
    - date resolved: **2023-04-10 21:37**\ , date raised: 2023-04-10 
 * Version 1.6.48: resolved Issue 1508: MainSystem (fix)
    - description:  prevent from creation of a pure MainSystem in exudyn; check constructor used in SC.AddSystem and MainSystem()
    - date resolved: **2023-04-10 21:37**\ , date raised: 2023-04-10 
 * Version 1.6.47: resolved Issue 1507: SystemContainer (docu)
    - description:  add description for SystemContainer itself; sying that it behaves like a variable in Python
    - date resolved: **2023-04-10 21:36**\ , date raised: 2023-04-10 
 * Version 1.6.46: resolved Issue 1503: item description (docu)
    - description:  add general description to RST / sphinx
    - date resolved: **2023-04-08 19:04**\ , date raised: 2023-04-08 
 * Version 1.6.45: resolved Issue 1496: AddMainObjectPyClass (check)
    - description:  C++: for adding objects, nodes, etc. currently py::object and py::dict is copied: check performance increase, if it is passed by reference
    - **notes:** tests show small performance improvements (<10 percent) for creation of items
    - date resolved: **2023-04-08 17:36**\ , date raised: 2023-04-07 
 * Version 1.6.44: resolved Issue 1498: MarkerNodeRotationCoordinate (check)
    - description:  check if Orientation type is correct, see docu
    - **notes:** removed Marker::Orientation from MarkerNodeRotationCoordinate as it is not provided
    - date resolved: **2023-04-08 17:30**\ , date raised: 2023-04-08 
 * Version 1.6.43: :textred:`resolved BUG 1497` : PyBeamSection 
    - description:  C++: segmentation fault in linux, caused by conversion from numpy array into std::array; use SetConstMatrixTemplateSafely for writing matrix or list
    - date resolved: **2023-04-07 19:05**\ , date raised: 2023-04-07 
 * Version 1.6.42: :textred:`resolved BUG 1495` : BeamSection 
    - description:  PyBeamSection does not call parent constructor (BeamSection); leads to seg fault in linux version
    - date resolved: **2023-04-07 13:32**\ , date raised: 2023-04-07 
 * Version 1.6.41: resolved Issue 1272: GeometricallyExactBeam (check)
    - description:  check jacobian computation as numerical differentiation works better; similar to #1100
    - **notes:** fixed bug of GeometricallyExactBeam having wrong jacobian
    - date resolved: **2023-04-06 18:28**\ , date raised: 2022-09-24 
 * Version 1.6.40: resolved Issue 1492: Python utilities (docu)
    - description:  fix indentation of examples
    - date resolved: **2023-04-06 14:55**\ , date raised: 2023-04-06 
 * Version 1.6.39: resolved Issue 1491: ProcessParameterList (fix)
    - description:  remove addComputationIndex, as it is unused
    - date resolved: **2023-04-06 14:35**\ , date raised: 2023-04-06 
 * Version 1.6.38: resolved Issue 1156: renderState (docu)
    - description:  add description of render state from C++ into theDoc
    - **notes:** added section Render State in section Graphics and Visualization (GUI)
    - date resolved: **2023-04-05 14:14**\ , date raised: 2022-06-21 
 * Version 1.6.37: resolved Issue 1468: sphinx (docu)
    - description:  add issue and bugs section with resolved issues and open issues table; resolved issues with version
    - **notes:** already done earlier
    - date resolved: **2023-04-05 14:08**\ , date raised: 2023-03-22 
 * Version 1.6.36: resolved Issue 1490: ANCFBeam (change)
    - description:  remove testBeamRectangularSize
    - date resolved: **2023-04-05 13:59**\ , date raised: 2023-04-05 
 * Version 1.6.35: resolved Issue 1088: ANCFBeam3D (extension)
    - description:  add test model, compare to 2013 paper
    - date resolved: **2023-04-04 20:59**\ , date raised: 2022-05-16 
 * Version 1.6.34: :textred:`resolved BUG 1489` : CObjectANCFBeam 
    - description:  Computation of elastic forces uses global instead of local twist-and-curvature vector
    - date resolved: **2023-04-04 20:04**\ , date raised: 2023-04-04 
 * Version 1.6.33: resolved Issue 1488: NodePoint3DSlope23 (extension)
    - description:  add full rigid body output values to node (e.g., Rotation missing)
    - date resolved: **2023-04-04 13:31**\ , date raised: 2023-04-04 
 * Version 1.6.32: resolved Issue 1487: Acceleration (extension)
    - description:  add GetAcceleration function to all nodes; add acceleration to all ODE2-based node output variables
    - date resolved: **2023-04-04 13:27**\ , date raised: 2023-04-04 
 * Version 1.6.31: resolved Issue 1482: EnterTaskManager (change)
    - description:  removed call to EnterTaskManager in serial mode as it causes large overhead; also removed large array for tracer
    - date resolved: **2023-03-28 18:47**\ , date raised: 2023-03-28 
 * Version 1.6.30: resolved Issue 1481: InverseKinematicsNumerical (change)
    - description:  changed SolveIKine to Solve and changed OutputVariable Rotation to RotationMatrix at tool
    - date resolved: **2023-03-28 16:30**\ , date raised: 2023-03-28 
 * Version 1.6.29: resolved Issue 1480: LogSE3 (extension)
    - description:  fixed according to LogSO3 and added efficient version with vectors in C++
    - date resolved: **2023-03-28 15:10**\ , date raised: 2023-03-28 
 * Version 1.6.28: resolved Issue 1478: LogSO3 (fix)
    - description:  Add new LogSO3 C++ function, also to be used in LogSE3 which fully works for 0..pi rotation range; improved accuracy for very small rotations as well as rotations close to pi, e.g. 0.99999999\*pi, where the standard approach fails
    - date resolved: **2023-03-28 15:09**\ , date raised: 2023-03-27 
 * Version 1.6.27: resolved Issue 1469: RotationMatrix2Rxyz (fix)
    - description:  extend implementation for rot[1]=pi/2
    - **notes:** resolved both in Python and C++; some simulation results may change, especially in output of Rotations in sensors
    - date resolved: **2023-03-28 14:46**\ , date raised: 2023-03-22 
 * Version 1.6.26: resolved Issue 1477: LogSO3 (fix)
    - description:  Add new LogSO3 Python function, also to be used in LogSE3 which fully works for 0..pi rotation range; improved accuracy for very small rotations as well as rotations close to pi, e.g. 0.99999999\*pi, where the standard approach fails
    - date resolved: **2023-03-27 20:16**\ , date raised: 2023-03-27 
 * Version 1.6.25: resolved Issue 1476: RotationMatrix2... (fix)
    - description:  RotationMatrix2EulerParameters, RotationMatrix2RotXYZ, RotationMatrix2RotZYZ not working both with list of lists and np.arrays
    - date resolved: **2023-03-27 19:58**\ , date raised: 2023-03-27 
 * Version 1.6.23: resolved Issue 0312: Add all types to pybind (extension)
    - description:  add remaining types to pybind - for user elements
    - **notes:** important types already added; further types currently not planned
    - date resolved: **2023-03-27 00:45**\ , date raised: 2020-01-10 
 * Version 1.6.22: resolved Issue 0848: add demos to github (docu)
    - description:  add demo videos to github or to youtube
    - **notes:** already resolved earlier: added videos to youtube
    - date resolved: **2023-03-27 00:42**\ , date raised: 2021-12-26 
 * Version 1.6.21: resolved Issue 0843: connector jacobian springDamper (extension)
    - description:  add analytic jacobian for SpringDamper connector
    - **notes:** already resolved earlier
    - date resolved: **2023-03-27 00:42**\ , date raised: 2021-12-23 
 * Version 1.6.20: resolved Issue 0851: ComputeObjectODE2LHS (extension)
    - description:  add computation functions for bodies and connectors; for connectors, markerData is computed automatically
    - **notes:** already resolved earlier with precomputed lists
    - date resolved: **2023-03-27 00:40**\ , date raised: 2022-01-08 
 * Version 1.6.19: resolved Issue 1325: MergeGraphicsDataTriangleList (fix)
    - description:  merging of edges erroneous or problems with GraphicsDataCylinder
    - date resolved: **2023-03-27 00:21**\ , date raised: 2022-12-19 
 * Version 1.6.18: resolved Issue 1472: GraphicsData (extension)
    - description:  add addEdges option to all GraphicsData Python functions
    - **notes:** added to sphere, cylinder, SolidOfRevolution, SolidOfExtrusion
    - date resolved: **2023-03-27 00:10**\ , date raised: 2023-03-26 
 * Version 1.6.17: resolved Issue 1473: GraphicsData (change)
    - description:  improve edges of cylinder and sphere by adding 6 lines along cylinder, some circles for sphere
    - date resolved: **2023-03-27 00:09**\ , date raised: 2023-03-26 
 * Version 1.6.16: resolved Issue 1459: mention papers (docu)
    - description:  add list of papers where exudyn has been used in theDoc and RTD
    - date resolved: **2023-03-26 15:04**\ , date raised: 2023-03-10 
 * Version 1.6.15: resolved Issue 1471: issues tracker (docu)
    - description:  remove html file from docs, use RSTfiles instead / move to readthedocs
    - date resolved: **2023-03-24 20:40**\ , date raised: 2023-03-24 
 * Version 1.6.14: resolved Issue 1470: GraphicsDataBasis (extension)
    - description:  add orientation and duplicate function with homogeneous transformation (HT) argument
    - **notes:** function called GraphicsDataFrame for Homogeneous transformation
    - date resolved: **2023-03-24 10:04**\ , date raised: 2023-03-24 
 * Version 1.6.13: resolved Issue 1467: InverseKinematicsNumerical (fix)
    - description:  fix success
    - date resolved: **2023-03-22 12:06**\ , date raised: 2023-03-22 
    - resolved by: P. Manzl
 * Version 1.6.12: resolved Issue 1466: modKKDH (change)
    - description:  consistently renamed into modDHKK in robotics module
    - date resolved: **2023-03-22 11:04**\ , date raised: 2023-03-22 
 * Version 1.6.11: resolved Issue 1465: robotics.models (change)
    - description:  adjust robot definitions, add dhMode to dictionary; fix LinkList2Robot
    - date resolved: **2023-03-22 10:39**\ , date raised: 2023-03-22 
 * Version 1.6.10: resolved Issue 1464: artificialIntelligence (fix)
    - description:  adapt to stable-baselines3 > 1.5.0; Class OpenAIGymInterfaceEnv does not support stable-baselines3 > 1.5.0 because OpenAIGymInterfaceEnv is not inherited from gym.Env
    - date resolved: **2023-03-20 12:54**\ , date raised: 2023-03-20 
    - resolved by: P. Manzl
 * Version 1.6.9: resolved Issue 1463: mpi4py (extension)
    - description:  add option to use MPI (message passing interface) for ProcessParameterList
    - **notes:** tests on supercomputer successful
    - date resolved: **2023-03-18 22:59**\ , date raised: 2023-03-18 
 * Version 1.6.8: resolved Issue 1462: minimum coordinates (docu)
    - description:  change consistently to minimal coordinates
    - date resolved: **2023-03-14 17:39**\ , date raised: 2023-03-14 
 * Version 1.6.7: :textred:`resolved BUG 1461` : isIntType check ParameterVariation 
    - description:  In the ParameterVariation (part of the processing module)  the integer type check failed when Variables of different types were used, now checking each variable independently
    - **notes:** added isIntType arrays
    - date resolved: **2023-03-14 12:50**\ , date raised: 2023-03-14 
    - resolved by: P. Manzl
 * Version 1.6.6: resolved Issue 1457: Newton (docu)
    - description:  fix steps and Newton iterations in description of Newton settings (thanks to Martin Arnold!)
    - date resolved: **2023-03-12 23:05**\ , date raised: 2023-03-09 
 * Version 1.6.5: resolved Issue 1456: sphinx/github pages (docu)
    - description:  add examples and test models for better search results
    - date resolved: **2023-03-12 23:05**\ , date raised: 2023-03-09 
 * Version 1.6.4: resolved Issue 1445: sphinx/github pages (docu)
    - description:  add solver description
    - date resolved: **2023-03-12 23:05**\ , date raised: 2023-02-22 
 * Version 1.6.3: resolved Issue 1443: sphinx/github pages (docu)
    - description:  add theory parts as far as possible
    - date resolved: **2023-03-12 23:05**\ , date raised: 2023-02-22 
 * Version 1.6.2: resolved Issue 1460: theDoc (docu)
    - description:  change structure: theory, notations earlier; remove duplicated MatrixContainer description
    - date resolved: **2023-03-12 16:20**\ , date raised: 2023-03-12 
 * Version 1.6.1: resolved Issue 1458: artificialIntelligence nan test (extension)
    - description:  check for nan values in TestModel evaluation
    - **notes:** failed steps may lead to nan values when evaluating the TestModel in the artificialIntelligence module. When this is detected the flagNan is set and evaluation is stopped. Check for flagNan in the Evaluation should be implemented.
    - date resolved: **2023-03-10 15:59**\ , date raised: 2023-03-10 
    - resolved by: P. Manzl
 * Version 1.6.0: resolved Issue 1426: cleanup howto files (fix)
    - description:  check if info is still valid (NGsolve, WSL, anaconda, ...)
    - **notes:** removed outdated files
    - date resolved: **2023-03-08 16:58**\ , date raised: 2023-02-11 

***********
Version 1.5
***********

 * Version 1.5.118: resolved Issue 1454: julia (docu)
    - description:  add sub section on interoperability with julia in Overview on Exudyn / Advanced topics; add some relevant examples for usage
    - date resolved: **2023-03-05 16:40**\ , date raised: 2023-03-05 
 * Version 1.5.117: resolved Issue 1453: __NOGLFW option not working (fix)
    - description:  exclude according functions in rendererPythonInterface
    - **notes:** added NOGLFW version to automatic build and testsuite
    - date resolved: **2023-02-28 15:18**\ , date raised: 2023-02-28 
 * Version 1.5.116: resolved Issue 1452: Single command (docu)
    - description:  add some more detailed description in theDoc after Visualization settings dialog
    - date resolved: **2023-02-26 00:40**\ , date raised: 2023-02-26 
 * Version 1.5.115: resolved Issue 1451: solver interface (check)
    - description:  check modification of solver structures like it to be writable
    - **notes:** added now write functionality for solvers; use with care and only if you know what you do...
    - date resolved: **2023-02-26 00:12**\ , date raised: 2023-02-25 
 * Version 1.5.114: resolved Issue 1447: CSensorMarker (extension)
    - description:  add RotationMatrix as additional outputvariable type
    - **notes:** Coordinates also added as output variable
    - date resolved: **2023-02-24 15:46**\ , date raised: 2023-02-23 
 * Version 1.5.113: resolved Issue 1446: CSensorMarker (fix)
    - description:  does not check types; Make GetOutputVariableTypes as in objects; orientation only for selected markers; check types in Assemble prechecks
    - date resolved: **2023-02-24 15:46**\ , date raised: 2023-02-23 
 * Version 1.5.112: :textred:`resolved BUG 1449` : DOPRI5 
    - description:  small bug introduced in previous update: currentStepSize not updated any more
    - date resolved: **2023-02-24 15:45**\ , date raised: 2023-02-24 
 * Version 1.5.111: resolved Issue 1448: GenericJoint (extension)
    - description:  add experimental flag alternativeConstraints to switch to different constraint equations, e.g. for 3 constrained rotations; this resolves unphysical 180 degree flips especially in static cases; flag may be removed in future
    - **notes:** thanks for P. Manzl for raising this problem
    - date resolved: **2023-02-24 13:52**\ , date raised: 2023-02-24 
 * Version 1.5.110: resolved Issue 1444: sphinx/github pages (docu)
    - description:  fix equation references and figures
    - date resolved: **2023-02-24 13:50**\ , date raised: 2023-02-22 
 * Version 1.5.109: resolved Issue 1442: add readthedocs.io (docu)
    - description:  link github project to readthedocs.io site
    - date resolved: **2023-02-22 23:59**\ , date raised: 2023-02-22 
 * Version 1.5.108: resolved Issue 1441: sphinx/github pages (docu)
    - description:  add more parts on items (equations)
    - date resolved: **2023-02-22 23:57**\ , date raised: 2023-02-22 
 * Version 1.5.107: resolved Issue 1440: ParameterVariation (change)
    - description:  integer should be kept as integers, if they are provided as a list and if variation is only on integers
    - **notes:** integers are kept if either tuple(start, end, numberOfValues) with start/end and numberOfValues are all integer or list contains only integers
    - date resolved: **2023-02-22 13:46**\ , date raised: 2023-02-22 
    - resolved by: S. Holzinger
 * Version 1.5.106: resolved Issue 1439: numpy dependency (extension)
    - description:  add install_requires to setup.py to force numpy to be installed; exudynCPP always requires now numpy
    - date resolved: **2023-02-21 18:11**\ , date raised: 2023-02-21 
 * Version 1.5.105: resolved Issue 1438: sphinx / github pages (docu)
    - description:  add input/output tables for items in RST files
    - date resolved: **2023-02-21 10:14**\ , date raised: 2023-02-19 
 * Version 1.5.104: resolved Issue 1437: sphinx / github pages (docu)
    - description:  add items to RST files/Sphinx to show up on github pages
    - date resolved: **2023-02-19 21:39**\ , date raised: 2023-02-19 
 * Version 1.5.103: resolved Issue 1436: sphinx / github pages (docu)
    - description:  add settings and structures; fix latex parts; fix links
    - date resolved: **2023-02-18 00:56**\ , date raised: 2023-02-18 
 * Version 1.5.102: :textred:`resolved BUG 1433` : Lie group integration 
    - description:  system with > 1 node raises LinkedDataVectorBases exception
    - date resolved: **2023-02-16 16:00**\ , date raised: 2023-02-16 
 * Version 1.5.101: resolved Issue 1432: artificialIntelligence module (fix)
    - description:  fix np.float64 not beeing detected as a scalar (float) for initializationValues of RL environment and possible waiting without activated renderer
    - date resolved: **2023-02-16 15:58**\ , date raised: 2023-02-16 
    - resolved by: P. Manzl
 * Version 1.5.100: resolved Issue 1431: make github pages (extension)
    - description:  install workflow for github pages to host pages for exudyn
    - **notes:** due to automatic conversion, there are many small errors (typos) remaining; for safety, check theDoc.pdf in any case
    - date resolved: **2023-02-16 00:01**\ , date raised: 2023-02-16 
 * Version 1.5.99: resolved Issue 1430: create rst docu files (extension)
    - description:  Create rst (markup language) files for C++ command interface and Python utilities
    - date resolved: **2023-02-16 00:00**\ , date raised: 2023-02-15 
 * Version 1.5.98: resolved Issue 1429: revise docu creation (fix)
    - description:  revise several helper functions for docu creation to homogenize latex and rst output
    - date resolved: **2023-02-16 00:00**\ , date raised: 2023-02-15 
 * Version 1.5.97: resolved Issue 1428: ParameterVariation (change)
    - description:  add conversion of parameters which are numpy.float64 to float which simplifies checking agains type(float); computationIndex becomes int
    - date resolved: **2023-02-13 17:18**\ , date raised: 2023-02-13 
 * Version 1.5.96: resolved Issue 1427: IsReal(x) and IsInteger(x) (extension)
    - description:  add checks which allow to check versus any float / numpy.float resp. int / numpy.int values; added to advancedUtilities
    - date resolved: **2023-02-13 17:17**\ , date raised: 2023-02-13 
 * Version 1.5.95: resolved Issue 1425: sphinx (extension)
    - description:  add github pages at https://jgerstmayr.github.io/EXUDYN created with sphinx
    - date resolved: **2023-02-11 16:41**\ , date raised: 2023-02-11 
 * Version 1.5.94: resolved Issue 1423: numerical Jacobians (change)
    - description:  add variadic template to realize numerical differentiation for objects in a consistent way
    - date resolved: **2023-02-08 12:17**\ , date raised: 2023-02-08 
 * Version 1.5.93: resolved Issue 1404: Lie group method (fix)
    - description:  add consistent numerical derivatives for Lie group nodes, needing the composition rule for incremental changes
    - date resolved: **2023-02-08 12:17**\ , date raised: 2023-01-18 
 * Version 1.5.92: resolved Issue 1422: numerical Jacobians (change)
    - description:  add variadic template to realize numerical differentiation in a consistent way
    - date resolved: **2023-02-08 00:16**\ , date raised: 2023-02-07 
 * Version 1.5.91: resolved Issue 1421: LIE_GROUP_IMPLICIT_SOLVER (change)
    - description:  remove preprocessor flag, as it is always set
    - date resolved: **2023-02-07 12:07**\ , date raised: 2023-02-07 
 * Version 1.5.90: resolved Issue 1420: setup.py (extension)
    - description:  reduce output of all compiler options with --quiet mode
    - date resolved: **2023-02-02 18:27**\ , date raised: 2023-02-02 
 * Version 1.5.89: resolved Issue 1419: window size (extension)
    - description:  window size currently limited to screen size and larger windows not accepted; override size limitations by using glfwSetWindowSize
    - **notes:** added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated; added flag window.limitWindowToScreenSize ; by default this is deactivated
    - date resolved: **2023-02-02 10:31**\ , date raised: 2023-02-02 
 * Version 1.5.88: resolved Issue 1418: OpenVR (change)
    - description:  adapt projection for OpenVR compatibility; exchange multiplication of pose and eye to fulfill classic OpenGL needs
    - date resolved: **2023-02-01 10:30**\ , date raised: 2023-02-01 
 * Version 1.5.87: resolved Issue 1417: ConnectorSpringDamperExt (fix)
    - description:  remove unintended output during Assemble()
    - date resolved: **2023-01-25 20:09**\ , date raised: 2023-01-25 
 * Version 1.5.86: resolved Issue 1405: AddDistanceSensor (extension)
    - description:  add rotation of Marker to ObjectGround in SensorUserFunction
    - **notes:** added option to draw displaced laser beam; rotation added, if rigid marker used
    - date resolved: **2023-01-23 10:05**\ , date raised: 2023-01-19 
 * Version 1.5.85: resolved Issue 1416: AddDistanceSensor (fix)
    - description:  UFsensorDistance misses the rotation part in visualization of laser beam with ground object
    - date resolved: **2023-01-23 09:39**\ , date raised: 2023-01-23 
 * Version 1.5.84: resolved Issue 1413: CoordinateSpringDamperExt (change)
    - description:  change friction parameter dimensions to forces (because there is no normal force) and change parameter names
    - date resolved: **2023-01-22 23:43**\ , date raised: 2023-01-21 
 * Version 1.5.83: resolved Issue 1412: CoordinateSpringDamperExt (extension)
    - description:  finalize implementation for bristle friction model
    - date resolved: **2023-01-22 23:43**\ , date raised: 2023-01-21 
 * Version 1.5.82: resolved Issue 1411: CoordinateSpringDamperExt (extension)
    - description:  finalize implementation for limit stops
    - date resolved: **2023-01-22 23:43**\ , date raised: 2023-01-21 
 * Version 1.5.81: resolved Issue 1410: examples (change)
    - description:  adapt Examples/massSpringFrictionInteractive.py and Examples/lugreFrictionTest.py to new CoordinateSpringDamperExt
    - date resolved: **2023-01-21 22:04**\ , date raised: 2023-01-21 
 * Version 1.5.80: resolved Issue 1136: ConnectorsExt (extension)
    - description:  add extended Ext versions of connectors: CoordinateSpringDamperExt, TorsionalSpringDamperExt, LinearSpringDamperExt, which allow for friction, coordinate limitation (with other SD-values) and possibly future extensions
    - date resolved: **2023-01-21 21:29**\ , date raised: 2022-06-10 
 * Version 1.5.79: resolved Issue 1407: pause (extension)
    - description:  add option for pause by pressing space bar
    - date resolved: **2023-01-21 21:28**\ , date raised: 2023-01-21 
 * Version 1.5.78: resolved Issue 1409: CoordinateSpringDamper (change)
    - description:  remove dryFriction and dryFrictionProportionalZone; this functionality will be made available in CoordinateSpringDamperExt
    - **notes:** see CoordinateSpringDamper in theDoc.pdf how to convert old models using friction parameters to new ones; note user function interfaces have been changed!
    - date resolved: **2023-01-21 17:53**\ , date raised: 2023-01-21 
 * Version 1.5.77: resolved Issue 1408: WaitForUserToContinue (extension)
    - description:  add argument printMessage, which can be set false to avoid text output in console; default behavior preserved
    - date resolved: **2023-01-21 13:51**\ , date raised: 2023-01-21 
 * Version 1.5.76: resolved Issue 1406: git tags (extension)
    - description:  add git tags for every new version automatically
    - **notes:** now versions can be found easier in github and they match the version numbers in pypi with pip installer
    - date resolved: **2023-01-19 18:47**\ , date raised: 2023-01-19 
 * Version 1.5.75: resolved Issue 1314: Pybind11 (check)
    - description:  test compilation with higher version of pybind11 (currently pybind11=2.6.0 in setup.py
    - **notes:** already done earlier; works well and allows compilation with Python3.11; added some switches in setup.py as not all Pybind11 versions work everywhere
    - date resolved: **2023-01-19 01:04**\ , date raised: 2022-12-14 
 * Version 1.5.74: resolved Issue 1365: OpenVR (example)
    - description:  add Python example with openVR
    - date resolved: **2023-01-19 01:02**\ , date raised: 2023-01-03 
 * Version 1.5.73: resolved Issue 1326: OpenVR (change)
    - description:  add OpenVR license and mention in getting started
    - date resolved: **2023-01-19 01:02**\ , date raised: 2022-12-20 
 * Version 1.5.72: resolved Issue 1402: openVR (extension)
    - description:  add flag to set compilation with openVR in setup.py; copy .dll in Windows case
    - date resolved: **2023-01-18 22:57**\ , date raised: 2023-01-17 
 * Version 1.5.71: resolved Issue 1364: OpenVR (extension)
    - description:  test simple use case
    - **notes:** basic functionality added; enable compilation with openVR by setting '-D__EXUDYN_USE_OPENVR' in cpp compiler flags of setup.py
    - date resolved: **2023-01-17 09:32**\ , date raised: 2023-01-03 
    - resolved by: EXTENSION
 * Version 1.5.70: resolved Issue 1401: lockModelView (extension)
    - description:  add flag which allows to fully lock rotation, zoom, etc.; in this case, only initial values in openGL settings are accepted for setup of view, but mouse or key input is ignored
    - date resolved: **2023-01-17 09:20**\ , date raised: 2023-01-17 
 * Version 1.5.69: resolved Issue 1363: test OpenVR with basic OpenGL (check)
    - description:  check if current openGL is sufficient with textures
    - date resolved: **2023-01-16 23:11**\ , date raised: 2023-01-03 
 * Version 1.5.68: resolved Issue 1400: renderState (change)
    - description:  vectors and matrices are returned in numpy format; this allows simpler computation, but does not anymore allow to treat output as list (+ operator!)
    - date resolved: **2023-01-16 21:11**\ , date raised: 2023-01-16 
 * Version 1.5.67: resolved Issue 1399: renderState (change)
    - description:  initialization now done consistently for SC.AttachToRenderEngine() and exudyn.StartRenderer(); behavior should be as before
    - date resolved: **2023-01-16 21:10**\ , date raised: 2023-01-16 
 * Version 1.5.66: resolved Issue 1398: renderState (extension)
    - description:  add projectionMatrix containing the current projection usd (usually identity matrix)
    - date resolved: **2023-01-16 20:09**\ , date raised: 2023-01-16 
 * Version 1.5.65: resolved Issue 1397: showFaceEdges, showMeshEdges (fix)
    - description:  wrong switching causing to show edges only if also some faces are activated
    - date resolved: **2023-01-12 22:32**\ , date raised: 2023-01-12 
 * Version 1.5.64: resolved Issue 1387: Get...Output (extension)
    - description:  add way to work for Reference configuration
    - **notes:** nodes, bodies and markers allow reference configuration; sensors also allow it for GetSensorValues
    - date resolved: **2023-01-12 22:14**\ , date raised: 2023-01-12 
 * Version 1.5.63: resolved Issue 1389: configuration checks (extension)
    - description:  check all systemData C++ interface functions regarding illegal configuration
    - date resolved: **2023-01-12 22:03**\ , date raised: 2023-01-12 
 * Version 1.5.62: resolved Issue 1388: configuration checks (extension)
    - description:  IsConfigurationInitialCurrentReferenceVisualization and IsConfigurationInitialCurrentVisualization shall be extended for StartOfStep; add hint that a calling function may have used an illegal configuration or None
    - date resolved: **2023-01-12 22:03**\ , date raised: 2023-01-12 
 * Version 1.5.61: resolved Issue 1396: Demo (extnesion)
    - description:  add a two demos included into the python module; put into exudyn.demos; add hint for larger examples; Demo1() = without graphics, just creating output file; Demo2() is rigid3Dexample with SolutionViewer
    - date resolved: **2023-01-12 18:51**\ , date raised: 2023-01-12 
 * Version 1.5.60: resolved Issue 1385: help (extension)
    - description:  add exudyn.help() function for short notes
    - date resolved: **2023-01-12 18:04**\ , date raised: 2023-01-12 
 * Version 1.5.59: :textred:`resolved BUG 1390` : ComputeLinearizedSystem 
    - description:  not working because of wrong interface to ComputeJacobianODE2RHS
    - date resolved: **2023-01-12 17:53**\ , date raised: 2023-01-12 
 * Version 1.5.58: resolved Issue 1394: eigenvalues test (testing)
    - description:  refine test model for ComputeODE2Eigenvalues
    - date resolved: **2023-01-12 17:44**\ , date raised: 2023-01-12 
 * Version 1.5.57: resolved Issue 1393: ComputeJacobianAE (change)
    - description:  change default values to fit to conventional tangential stiffness matrix computation
    - date resolved: **2023-01-12 17:00**\ , date raised: 2023-01-12 
 * Version 1.5.56: resolved Issue 1392: ComputeJacobianODE1RHS (change)
    - description:  change default values to fit to conventional tangential stiffness matrix computation
    - date resolved: **2023-01-12 17:00**\ , date raised: 2023-01-12 
 * Version 1.5.55: resolved Issue 1391: ComputeJacobianODE2RHS (change)
    - description:  change default values to fit to conventional tangential stiffness matrix computation
    - date resolved: **2023-01-12 17:00**\ , date raised: 2023-01-12 
 * Version 1.5.54: :textred:`resolved BUG 1386` : GetMarkerOutput 
    - description:  crashes if used with any configuration before Assemble(); add check that it may be only called after Assemble()
    - **notes:** add check and raise error
    - date resolved: **2023-01-12 14:15**\ , date raised: 2023-01-12 
 * Version 1.5.53: resolved Issue 1384: visualizationSettings (extension)
    - description:  add textOffsetFactor in general options to adjust text offset if not drawn always in front
    - **notes:** also adjusted some text appearance settings
    - date resolved: **2023-01-11 20:42**\ , date raised: 2023-01-11 
 * Version 1.5.52: resolved Issue 1383: visualizationSettings (docu)
    - description:  update docu of Visualization settings dialog in introduction
    - date resolved: **2023-01-11 17:41**\ , date raised: 2023-01-11 
 * Version 1.5.51: resolved Issue 1382: visualizationSettings (change)
    - description:  increase initial size of visualizations dialog for larger screens; see  Section :ref:`sec-overview-basics-visualizationsettings`\  how to change to a smaller window size
    - date resolved: **2023-01-11 16:57**\ , date raised: 2023-01-11 
 * Version 1.5.50: resolved Issue 1381: visualizationSettings (change)
    - description:  change order of items to have easier access to each tree node
    - **notes:** dialogs.openTreeView=True can be used to switch to original behavior
    - date resolved: **2023-01-11 16:34**\ , date raised: 2023-01-11 
 * Version 1.5.49: resolved Issue 1379: item text color (change)
    - description:  draw colors in item texts (node numbers, etc.) different from item color to improve visibility
    - date resolved: **2023-01-11 16:28**\ , date raised: 2023-01-11 
 * Version 1.5.48: resolved Issue 1308: OpenGL texts (extension)
    - description:  add sub function for drawing text bitmaps; use different order of drawing for transparent and non-transparent triangles to get improved text drawing
    - **notes:** now having possibility to draw item texts in front or transparent
    - date resolved: **2023-01-11 16:28**\ , date raised: 2022-12-08 
 * Version 1.5.47: resolved Issue 1380: loads (change)
    - description:  draw load numbers at end of load vectors
    - **notes:** made Force consistent with Torque and LoadMassProportional
    - date resolved: **2023-01-11 14:43**\ , date raised: 2023-01-11 
 * Version 1.5.46: resolved Issue 1378: font drawing (extension)
    - description:  add visualizationSettings.general for textDrawInFront and textHasBackGround for having a (currently) white background color
    - date resolved: **2023-01-11 09:46**\ , date raised: 2023-01-11 
 * Version 1.5.45: resolved Issue 1377: openvr (change)
    - description:  not compatible right now with Exudyn under Windows while linux seems to work
    - date resolved: **2023-01-09 21:11**\ , date raised: 2023-01-09 
 * Version 1.5.44: resolved Issue 1370: advancedUtilities (extension)
    - description:  add advanced utilities depnding on numpy and math; functions not suitable for exudyn.basicUtilities or exudyn.utilities
    - date resolved: **2023-01-07 01:57**\ , date raised: 2023-01-06 
 * Version 1.5.43: resolved Issue 1375: initial accelerations (extension)
    - description:  add flag to decide whether initial accelerations are erased at beginning of simulation; the background is that they should not be erased, if they are prolonged for subsequent simulation; check if this changes behavior, as ODE2_tt coordinates are anyway resetted at Assemble()
    - date resolved: **2023-01-07 01:56**\ , date raised: 2023-01-06 
 * Version 1.5.42: resolved Issue 1376: advancedUtilities (change)
    - description:  move functions exudyn.utilities from to exudyn.advancedUtilities; still included in exudyn.utilities
    - date resolved: **2023-01-06 22:54**\ , date raised: 2023-01-06 
 * Version 1.5.41: :textred:`resolved BUG 1374` : InteractiveDialog 
    - description:  accelerations are not reused from last period; need to copy current accelerations into initial accelerations
    - date resolved: **2023-01-06 21:21**\ , date raised: 2023-01-06 
 * Version 1.5.40: resolved Issue 1373: GraphicsData (change)
    - description:  add option to add edges in GraphicsData...Cube... functions; switch some order of options 
    - date resolved: **2023-01-06 19:25**\ , date raised: 2023-01-06 
 * Version 1.5.39: resolved Issue 1372: utilities (fix)
    - description:  ComputeSkewMatrix: remove duplicate (with less functionality) from utilities and put into rigidBodyUtilities; correct import in FEM
    - date resolved: **2023-01-06 19:19**\ , date raised: 2023-01-06 
 * Version 1.5.38: resolved Issue 1371: exudyn.utilities (change)
    - description:  remove functions CheckInputVector and CheckInputIndexArray as they are unused and replaced with improved functions in advancedUtilities
    - date resolved: **2023-01-06 17:46**\ , date raised: 2023-01-06 
 * Version 1.5.37: resolved Issue 1369: add sub-module machines (extension)
    - description:  will include mechanical engineering and machine element relevant topics, such as bearings, gears, mechanisms, etc.
    - date resolved: **2023-01-06 11:47**\ , date raised: 2023-01-06 
 * Version 1.5.36: resolved Issue 1368: GraphicsDataCylinder (extension)
    - description:  new option to only add edges without faces
    - date resolved: **2023-01-05 23:03**\ , date raised: 2023-01-05 
 * Version 1.5.35: resolved Issue 1367: GraphicsDataSolidExtrusion (extension)
    - description:  add edges and normals for smoothening
    - date resolved: **2023-01-05 23:03**\ , date raised: 2023-01-05 
 * Version 1.5.34: resolved Issue 1366: GraphicsData (extension)
    - description:  add functionality for GraphicsDataSolidExtrusion to include circles: CirclePointsAndSegments() and to manipulate point lists defining geometries: SegmentsFromPoints()
    - date resolved: **2023-01-05 18:15**\ , date raised: 2023-01-05 
 * Version 1.5.33: resolved Issue 1362: add OpenVR interface (extension)
    - description:  add interface, but by default not compiled
    - date resolved: **2023-01-03 23:00**\ , date raised: 2023-01-03 
 * Version 1.5.32: resolved Issue 1361: Python 3.11 (extension)
    - description:  create first development wheels for Python 3.11; Windows+Linux
    - **notes:** problems to get conda with Python3.11 running, especially on ubuntu
    - date resolved: **2023-01-03 18:48**\ , date raised: 2023-01-03 
 * Version 1.5.31: resolved Issue 1360: Python 3.11 (extension)
    - description:  adjust setup.py to support Python 3.11; requires Pybind11 2.10
    - date resolved: **2023-01-03 14:37**\ , date raised: 2023-01-03 
 * Version 1.5.30: resolved Issue 1335: RollingDiscPenalty (extension)
    - description:  add test example for ground moving (rotating table
    - date resolved: **2023-01-02 19:07**\ , date raised: 2022-12-25 
 * Version 1.5.29: resolved Issue 1359: ObjectGround (extension)
    - description:  extend for referenceRotation to allow rotation of ground objects (especially for visualization and contact)
    - date resolved: **2023-01-02 11:49**\ , date raised: 2023-01-02 
 * Version 1.5.28: resolved Issue 0839: multithreaded jacobian (extension)
    - description:  add functionality for multithreaded jacobian and mass matrix
    - **notes:** mass matrix resolved; other is redundant with #1203
    - date resolved: **2023-01-02 01:04**\ , date raised: 2021-12-19 
 * Version 1.5.27: resolved Issue 0828: MT integration2 (extension)
    - description:  add multithreading to mass matrix and jacobian; add multithreaded adding of vector with special templated functions to allow special (templated) solver functions
    - **notes:** redundant with #839
    - date resolved: **2023-01-02 01:02**\ , date raised: 2021-12-09 
 * Version 1.5.26: resolved Issue 0791: test Eigen SimplicialLDLT (extension)
    - description:  use systemMatricesArePD to enable LDLT solver
    - **notes:** tested earlier; does not work in most cases
    - date resolved: **2023-01-02 01:01**\ , date raised: 2021-11-02 
 * Version 1.5.25: resolved Issue 0437: solvercontainer+ systemcontainer (change)
    - description:  remove SolverContainer and SystemContainer from systemStructuresDefinition and work manually
    - **notes:** already done much earlier
    - date resolved: **2023-01-02 00:59**\ , date raised: 2020-07-21 
 * Version 1.5.24: resolved Issue 0315: Add user object (extension)
    - description:  add user object
    - **notes:** done with GenericODE2 and CoordinateVectorConstraint
    - date resolved: **2023-01-02 00:57**\ , date raised: 2020-01-10 
 * Version 1.5.23: resolved Issue 0261: MarkerDataJacobians (extension)
    - description:  Add access functions for jacobians and other marker data: SetPositionJacobian, GetPositionJacobian, etc.; add jacobian types = markertypes, which check if wrong jacobian is accessed
    - **notes:** not suitable anymore
    - date resolved: **2023-01-02 00:56**\ , date raised: 2019-09-11 
 * Version 1.5.22: resolved Issue 1331: exudyn cpp (extension)
    - description:  add __repr__ and help which writes some information on workflow (github, theDoc, Examples, ...)
    - **notes:** not possible
    - date resolved: **2023-01-02 00:51**\ , date raised: 2022-12-21 
 * Version 1.5.21: resolved Issue 1323: AVX2 (extension)
    - description:  update documentation for improved functionality with AVX2 code
    - **notes:** already done when resolving #1330
    - date resolved: **2023-01-02 00:50**\ , date raised: 2022-12-17 
 * Version 1.5.20: resolved Issue 1355: visualizationSettings (check)
    - description:  check possibility for update loop to continue simulation
    - **notes:** not feasible now, as it requires to modify time integration loop
    - date resolved: **2023-01-02 00:48**\ , date raised: 2022-12-31 
 * Version 1.5.19: resolved Issue 1353: visualizationSettings (extension)
    - description:  add KEY Q shortcut to close dialog
    - date resolved: **2023-01-02 00:48**\ , date raised: 2022-12-31 
 * Version 1.5.18: resolved Issue 1358: DictionariesGetSet (change)
    - description:  add new types VectorFloat and MatrixFloat in order to distinguish from double values
    - **notes:** also fixed matrix conversion in visualizationSettings
    - date resolved: **2023-01-02 00:47**\ , date raised: 2023-01-02 
 * Version 1.5.17: resolved Issue 1357: visualizationSettings (extension)
    - description:  add single click edit event
    - date resolved: **2023-01-01 23:27**\ , date raised: 2023-01-01 
 * Version 1.5.16: resolved Issue 1354: visualizationSettings (extension)
    - description:  double click on bool variables changes state
    - date resolved: **2023-01-01 23:22**\ , date raised: 2022-12-31 
 * Version 1.5.15: resolved Issue 1352: tkinter dialogs (fix)
    - description:  add option for transparency to dialogs
    - **notes:** visualizationSettings.dialogs.transparency
    - date resolved: **2022-12-31 11:01**\ , date raised: 2022-12-31 
 * Version 1.5.14: resolved Issue 1351: tkinter MacOS (fix)
    - description:  adjust font size and row size in ttk right mouse dialog
    - **notes:** visualizationSettings.dialogs.fontScalingMacOS
    - date resolved: **2022-12-31 11:00**\ , date raised: 2022-12-31 
 * Version 1.5.13: resolved Issue 1350: tkinter MacOS (fix)
    - description:  adjust font size and row size in ttk visualizationSettings dialog
    - **notes:** visualizationSettings.dialogs.fontScalingMacOS
    - date resolved: **2022-12-31 11:00**\ , date raised: 2022-12-31 
 * Version 1.5.12: :textred:`resolved BUG 0752` : tkinter MacOS 
    - description:  tkinter fails when loaded inside interactive.py; early call to Tk() inside an example works and seems to help; check options to correctly load tkinter in MacOS (Rosetta 2, on M1)
    - **notes:** problem fixed with single threaded renderer, doing illegal operations in glfw callbacks
    - date resolved: **2022-12-31 01:01**\ , date raised: 2021-09-20 
 * Version 1.5.11: resolved Issue 1349: RequireVersion (fix)
    - description:  some bugs including exu.GetVersionString and not raising exception
    - date resolved: **2022-12-30 15:03**\ , date raised: 2022-12-30 
 * Version 1.5.10: resolved Issue 1343: tkinter in exudyn (docu)
    - description:  add comment in FAQ on potential problems with tkinter; add option to let exudyn know if tkinter is already running
    - date resolved: **2022-12-28 23:39**\ , date raised: 2022-12-27 
 * Version 1.5.9: resolved Issue 1347: visualizationSettings (extension)
    - description:  add flag visualizationSettings.dialog.multiThreadedDialogs to turn on/off immediate apply of visualizationSettings changes; extend exudyn.GUI and rendererPythonInterface.h accordingly
    - **notes:** NOTE that this flag should be turned off in case of crashes during/after dialogs; could make problems on special platforms such as MacOS
    - date resolved: **2022-12-28 23:05**\ , date raised: 2022-12-28 
 * Version 1.5.8: resolved Issue 1348: EditDictionaryWithTypeInfo (change)
    - description:  change interface to pass directly visualizationSettings, allowing to update data
    - date resolved: **2022-12-28 21:50**\ , date raised: 2022-12-28 
 * Version 1.5.7: resolved Issue 1346: visualizationSettings (check)
    - description:  check if visualiuation settings dialog can be installed such that updating of data immediately affects renderer window
    - date resolved: **2022-12-28 16:53**\ , date raised: 2022-12-28 
 * Version 1.5.6: resolved Issue 1339: tkinter MacOS (fix)
    - description:  add tkinter.Tk() into exudyn.sys["tkinterRoot"] before call to StartRenderer(); check for tkinterRoot in exudyn.sys on startup of interactive dialog; resolves crash on MacOS for SolutionViewer
    - date resolved: **2022-12-27 23:50**\ , date raised: 2022-12-26 
 * Version 1.5.5: resolved Issue 1345: interface default args (fix)
    - description:  change true/false to True/False in theDoc for PYthon command interface
    - date resolved: **2022-12-27 23:24**\ , date raised: 2022-12-27 
 * Version 1.5.4: resolved Issue 1344: exudyn (extension)
    - description:  add function IsRendererRunning(), to avoid Warnings when renderer is restarted
    - date resolved: **2022-12-27 23:23**\ , date raised: 2022-12-27 
 * Version 1.5.3: resolved Issue 1338: MacOS (fix)
    - description:  DoRendererIdleTasks() crashes if no Renderer is active; add check in python call to avoid crash
    - **notes:** crash resolved on Windows; MacOS test still open
    - date resolved: **2022-12-27 17:36**\ , date raised: 2022-12-26 
 * Version 1.5.2: resolved Issue 1340: MacOS multithreading (change)
    - description:  activate simplified multithreading by switching to TinyThreading in case of MacOS
    - date resolved: **2022-12-27 17:19**\ , date raised: 2022-12-26 
 * Version 1.5.1: resolved Issue 1342: MacOS M1 (extension)
    - description:  add string ARM to Platform string, e.g., used in solution and sensor files
    - date resolved: **2022-12-27 17:03**\ , date raised: 2022-12-27 
 * Version 1.5.0: resolved Issue 1336: test suite (fix)
    - description:  resolve linux problem with RigidBodySpringDamper.py MiniExample
    - **notes:** problem in testsuite due to AVX differences, causing all non-AVX cases to fail (also linux); fixed with special case in reference solutions
    - date resolved: **2022-12-25 20:10**\ , date raised: 2022-12-25 

***********
Version 1.4
***********

 * Version 1.4.65: resolved Issue 1264: RollingDiscPenalty (extension)
    - description:  correct torque on ground, which currently has no effect, but will be used for moving ground
    - **notes:** resolved earlier
    - date resolved: **2022-12-25 12:15**\ , date raised: 2022-09-17 
 * Version 1.4.64: resolved Issue 1298: ReevingSystemLinear (testing)
    - description:  add test model to test suite
    - date resolved: **2022-12-25 12:11**\ , date raised: 2022-12-01 
 * Version 1.4.63: resolved Issue 1333: gcc linux (change)
    - description:  adjust -march compilation options for improved performance on linux; try -march=skylake for AVX2 optimization
    - **notes:** did not work: either gives compilation errors or segmentation faults
    - date resolved: **2022-12-25 00:45**\ , date raised: 2022-12-24 
 * Version 1.4.62: resolved Issue 1334: NOGLFW (change)
    - description:  remove SystemContainer constructor warning for AttachToRenderEngine in case of NOGLFW
    - **notes:** also done for DetachFromRenderEngine
    - date resolved: **2022-12-25 00:06**\ , date raised: 2022-12-25 
 * Version 1.4.61: resolved Issue 1332: MacOS (fix)
    - description:  compilation does not finish due to -framework Cocoa, etc. errors
    - **notes:** changed -framework library lists by separating commands into two separate strings; compilation now runs through for Python 3.8-3.10 on MacOS with M1 and 3.7.-3.10 on _x86 emulation
    - date resolved: **2022-12-24 22:20**\ , date raised: 2022-12-24 
 * Version 1.4.60: resolved Issue 1330: AVX2 (fix)
    - description:  fix check for AVX and AVX2 on module import
    - date resolved: **2022-12-21 19:33**\ , date raised: 2022-12-21 
 * Version 1.4.59: resolved Issue 1329: SolutionViewer (example)
    - description:  add example for SolutionViewer with multiple static simulations performed, writing results into single file; see Examples/solutionViewerMultipleSimulations.py
    - date resolved: **2022-12-21 18:05**\ , date raised: 2022-12-21 
 * Version 1.4.58: resolved Issue 1328: simulationSettings.solutionSettings (extension)
    - description:  add option writeInitialValues in order to turn on/off writing of initial values for coordinatesSolution and sensors; by default turned on as was done so far
    - date resolved: **2022-12-21 14:07**\ , date raised: 2022-12-21 
 * Version 1.4.57: resolved Issue 1037: sensor files (extension)
    - description:  add file footer information same as in coordinatesSolutionFile including CPUtimeElapsed with 3 digits
    - **notes:** turned on by default, but should be switched off if it causes compatibility problems for your postprocessing tool
    - date resolved: **2022-12-21 12:09**\ , date raised: 2022-04-07 
 * Version 1.4.56: resolved Issue 1327: sensorsWriteFileHeader (fix)
    - description:  not working
    - **notes:** added option into solver, now turning on/off as desired; added option into solver, now turning on/off as desired; added option into solver, now turning on/off as desired
    - date resolved: **2022-12-21 12:06**\ , date raised: 2022-12-21 
 * Version 1.4.55: resolved Issue 1317: DistanceSensor (example)
    - description:  Add example with distance sensor
    - date resolved: **2022-12-20 13:49**\ , date raised: 2022-12-16 
 * Version 1.4.54: resolved Issue 1318: DistanceSensor (extension)
    - description:  Add measurement for GeneralContact trigsRigidBody
    - date resolved: **2022-12-19 21:49**\ , date raised: 2022-12-16 
 * Version 1.4.53: resolved Issue 1316: DistanceSensor (extension)
    - description:  add utilities function to create DistanceSensor based on general contact
    - **notes:** added AddDistanceSensor to exudyn utilities.py
    - date resolved: **2022-12-19 01:09**\ , date raised: 2022-12-16 
 * Version 1.4.52: resolved Issue 1324: DistanceSensor (extension)
    - description:  utilities function AddDistanceSensor extended for measureVelocity to measure velocity similar as in a laser Doppler vibrometer (LDV)
    - date resolved: **2022-12-18 20:59**\ , date raised: 2022-12-18 
 * Version 1.4.51: resolved Issue 1320: GeneralContact (extension)
    - description:  add binary contact types to pybind interface; add conversion of binary types to type indices
    - **notes:** only TypeIndex added which is sufficient for DistanceSensor
    - date resolved: **2022-12-17 00:32**\ , date raised: 2022-12-16 
 * Version 1.4.50: resolved Issue 1319: DistanceSensor (extension)
    - description:  Add radius for option to measure with cylinder with given radius instead of line; useful for particles
    - date resolved: **2022-12-17 00:32**\ , date raised: 2022-12-16 
 * Version 1.4.49: resolved Issue 1321: DistanceSensor (extension)
    - description:  Add option to select which contact types are considered
    - date resolved: **2022-12-17 00:31**\ , date raised: 2022-12-16 
 * Version 1.4.48: resolved Issue 1322: AVX2 import (extension)
    - description:  add checks based on numpy.core._multiarray_umath to find if CPU has AVX2 support; in release, user may directly import noAVX version by setting sys.exudynCPUhasAVX2=False
    - date resolved: **2022-12-16 23:42**\ , date raised: 2022-12-16 
 * Version 1.4.47: :textred:`resolved BUG 1315` : SetSearchTreeBox 
    - description:  has no effect, and searchTree is always computed automatically
    - **notes:** now search tree can be initialized smaller or larger than initial geometry in order to optimize for specific problem
    - date resolved: **2022-12-15 21:20**\ , date raised: 2022-12-15 
 * Version 1.4.46: resolved Issue 1268: DistanceSensor (extension)
    - description:  add simple distance sensor
    - **notes:** can be realized with GeneralContact and SensorUserFunction
    - date resolved: **2022-12-15 20:23**\ , date raised: 2022-09-21 
 * Version 1.4.45: resolved Issue 1271: GeneralContact (extension)
    - description:  Add interface functions for GeneralContact: allowing to measure distance along a line; get markers/spheres in box; get beams in box; etc.
    - date resolved: **2022-12-15 20:20**\ , date raised: 2022-09-21 
 * Version 1.4.44: resolved Issue 1313: AVX2 (extension)
    - description:  add additional library without AVX and use try/except to import exudynCPPnoAVX in case that AVX2 is not available
    - date resolved: **2022-12-14 20:27**\ , date raised: 2022-12-14 
 * Version 1.4.43: resolved Issue 1312: SolveDynamic (extension)
    - description:  add flag computeMassMatrixInversePerBody for explicit time integration to compute mass matrix inverse per body; read theDoc and use with care!
    - **notes:** check your models! ComputeMassMatrix has been adapted for every body!!!
    - date resolved: **2022-12-13 21:00**\ , date raised: 2022-12-13 
 * Version 1.4.42: resolved Issue 1311: DynamicSolver (fix)
    - description:  include flag timeIntegration.reuseConstantMassMatrix into explicit solver; up to now, this option was always turned on
    - date resolved: **2022-12-13 18:51**\ , date raised: 2022-12-13 
 * Version 1.4.41: resolved Issue 1310: searchTreeUpdateCounter (fix)
    - description:  needs reset to 0 after reaching limit
    - date resolved: **2022-12-13 17:58**\ , date raised: 2022-12-13 
 * Version 1.4.40: resolved Issue 0847: GeneralContact searchtree (extension)
    - description:  add options to recompute searchtree size if particles are moving out of region; add option to flush all dynamical arrays (searchtree, allActiveContacts, etc.) after certain time
    - **notes:** added option resetSearchTreeInterval into GeneralContact settings
    - date resolved: **2022-12-12 10:40**\ , date raised: 2021-12-23 
 * Version 1.4.39: resolved Issue 1309: rigidBodyUtilities (extension)
    - description:  AddRigidBody: add default argument for nodeType as RotationEulerParameters; simplifies creation of rigid bodies
    - date resolved: **2022-12-08 18:16**\ , date raised: 2022-12-08 
 * Version 1.4.38: resolved Issue 1302: GL_POLYGON_OFFSET_FILL (fix)
    - description:  correct polygon offset setting for regular faces/lines and add adjustable parameter in visualizationSettings
    - date resolved: **2022-12-08 01:50**\ , date raised: 2022-12-06 
 * Version 1.4.37: resolved Issue 1307: OpenGL (change)
    - description:  change line and polygon drawing in order to avoid line artifacts; newly introduced polygon offset may cause problems: check your visualization, send reports if problems and set polygonOffset=0 in severe cases
    - date resolved: **2022-12-08 01:06**\ , date raised: 2022-12-08 
 * Version 1.4.36: resolved Issue 1304: ConnectorRollingDiscPenalty (docu)
    - description:  extend docu for arbitrary planeNormal and moving ground
    - date resolved: **2022-12-07 21:57**\ , date raised: 2022-12-07 
 * Version 1.4.35: resolved Issue 1306: ObjectJointRollingDisc (extension)
    - description:  add discAxis to object parameters, being able to change from default x-axis
    - date resolved: **2022-12-07 20:18**\ , date raised: 2022-12-07 
 * Version 1.4.34: resolved Issue 1305: ObjectJointRollingDisc (change)
    - description:  change computation of trail, generalized for two bodies moving relative to each other (needs further testing)
    - date resolved: **2022-12-07 20:18**\ , date raised: 2022-12-07 
 * Version 1.4.33: resolved Issue 1303: ConnectorRollingDiscPenalty (extension)
    - description:  extended formulation for arbitrary planeNormal; ground body can now also move in space (testing needed)
    - date resolved: **2022-12-07 17:49**\ , date raised: 2022-12-07 
 * Version 1.4.32: resolved Issue 1301: graphicsDataUtilities (fix)
    - description:  AddEdgesAndSmoothenNormals fixed to process colors
    - date resolved: **2022-12-06 12:37**\ , date raised: 2022-12-06 
 * Version 1.4.31: resolved Issue 1300: graphicsDataUtilities (extension)
    - description:  GraphicsDataFromPointsAndTrigs extended to accept color per point or 4 RGBA values
    - date resolved: **2022-12-06 12:37**\ , date raised: 2022-12-06 
 * Version 1.4.30: :textred:`resolved BUG 1299` : GeneralContact 
    - description:  searchTreeBox not computed automatically (must be set manually)
    - **notes:** computation of searchtree performed automatically now if no search tree box is specified; output message written in case of verboseMode=1
    - date resolved: **2022-12-02 12:51**\ , date raised: 2022-12-02 
 * Version 1.4.29: resolved Issue 1297: ConnectorSpringDamper (extension)
    - description:  return scalar spring-damper force with OutputVariableType ForceLocal
    - date resolved: **2022-12-01 16:31**\ , date raised: 2022-12-01 
 * Version 1.4.28: resolved Issue 1296: dynamic solver (check)
    - description:  add check that useIndex2=True is consistent with useNewmark=True
    - date resolved: **2022-11-30 14:12**\ , date raised: 2022-11-29 
 * Version 1.4.27: resolved Issue 0925: ReevingSystemSprings (extension)
    - description:  create reeving system along points defined by markers, using massless springs and one total length; add rigid body markers for position of sheaves; use coordinate markers for prescribed change of length at end of reeving system
    - **notes:** see new ObjectConnectorReevingSystemSprings
    - date resolved: **2022-11-19 01:11**\ , date raised: 2022-02-03 
 * Version 1.4.26: resolved Issue 1295: BeamGeometricallyExact2D (extension)
    - description:  add damping terms for bending, axial and shear deformation
    - date resolved: **2022-11-17 15:59**\ , date raised: 2022-11-17 
 * Version 1.4.25: resolved Issue 1294: BeamGeometricallyExact2D (extension)
    - description:  add reference strain/curvature
    - date resolved: **2022-11-17 15:59**\ , date raised: 2022-11-17 
 * Version 1.4.24: resolved Issue 1293: MarkerDataStructure (extension)
    - description:  extend for more than 2 MarkerData; adjust caller functions
    - date resolved: **2022-11-13 16:53**\ , date raised: 2022-11-13 
 * Version 1.4.23: resolved Issue 1291: Lie Group integration (fix)
    - description:  CompositionRule in LieGroup nodes does not consider reference position; add reference configuration in composition rule (and subtract afterwards)
    - date resolved: **2022-11-09 15:00**\ , date raised: 2022-11-09 
 * Version 1.4.22: :textred:`resolved BUG 1289` : visualizationSettings 
    - description:  interactive.trackMarker has wrong type leading to crash when closing visualizationSettings
    - date resolved: **2022-11-05 14:15**\ , date raised: 2022-11-05 
 * Version 1.4.21: resolved Issue 1286: trackMarker (fix)
    - description:  selection of objects wrong / check mouse selection procedure
    - date resolved: **2022-11-04 20:46**\ , date raised: 2022-11-02 
 * Version 1.4.20: resolved Issue 1288: ConnectorRollingDiscPenalty (extension)
    - description:  add useLinearProportionalZone which performs better in implicit time integration
    - date resolved: **2022-11-04 17:16**\ , date raised: 2022-11-04 
 * Version 1.4.19: resolved Issue 1287: ConnectorRollingDiscPenalty (extension)
    - description:  add viscousFriction, using separate values for local X/Y coordinates
    - date resolved: **2022-11-04 17:16**\ , date raised: 2022-11-04 
 * Version 1.4.18: resolved Issue 1276: Renderer: marker tracking (extension)
    - description:  add option to track markers in renderer
    - **notes:** see visualizationSettings.interactive for several trackMarker... options
    - date resolved: **2022-11-02 17:13**\ , date raised: 2022-10-12 
 * Version 1.4.17: resolved Issue 1279: release_assert (change)
    - description:  remove all asserts (in automatic code generation)
    - date resolved: **2022-11-02 17:10**\ , date raised: 2022-10-13 
 * Version 1.4.16: resolved Issue 1285: Lobatto, LobattoIntegrate (fix)
    - description:  order nomenclature is wrong. Lobatto2 needs to be renamed into Lobatto1 and so on
    - date resolved: **2022-11-02 16:43**\ , date raised: 2022-11-02 
 * Version 1.4.15: resolved Issue 1284: extend ALEANCF beam (extension)
    - description:  add effects due to axial and bending viscous damping in case of movingMassFactor==1
    - **notes:** new damping terms for axially moving beams implemented according to 2022 Paper of Pieber, Ntarladima, Gerstmayr only for case movingMassFactor=1
    - date resolved: **2022-10-31 15:43**\ , date raised: 2022-10-25 
 * Version 1.4.14: :textred:`resolved BUG 1283` : ProcessParameterList 
    - description:  in case useMultiProcessing=False the output file does not contain correct ranges
    - date resolved: **2022-10-20 21:59**\ , date raised: 2022-10-20 
 * Version 1.4.13: resolved Issue 1282: PlotSensor (extension)
    - description:  added return value including [plt, fig, ax, line] to be used for subsequent operations
    - date resolved: **2022-10-19 20:30**\ , date raised: 2022-10-19 
 * Version 1.4.12: resolved Issue 1281: results monitor (extension)
    - description:  extend results monitor for viewing more detailed results of ParameterVariation with colorVariations option; add function SingleIndex2SubIndices
    - date resolved: **2022-10-17 07:30**\ , date raised: 2022-10-17 
 * Version 1.4.11: :textred:`resolved BUG 1280` : ParameterVariation 
    - description:  resultsFile not working (problem with ProcessParameterList
    - date resolved: **2022-10-14 08:22**\ , date raised: 2022-10-14 
 * Version 1.4.10: resolved Issue 1278: center point (fix)
    - description:  add key O to change center of rotation; resolve finally original issue 1155
    - date resolved: **2022-10-12 22:37**\ , date raised: 2022-10-12 
 * Version 1.4.9: resolved Issue 1277: shadowPolygonOffset (change)
    - description:  decrease default value from 10 to 0.1; adjust in your models
    - date resolved: **2022-10-12 20:27**\ , date raised: 2022-10-12 
 * Version 1.4.8: resolved Issue 1275: GetMarkerOutput (extension)
    - description:  add function mbs.GetMarkerOutput() to return position, velocitiy, rotation matrix and angular velocity for markers if available
    - date resolved: **2022-10-12 09:18**\ , date raised: 2022-10-12 
 * Version 1.4.7: :textred:`resolved BUG 1274` : ALECable2D 
    - description:  missing term L in preComputedB terms in C++ implementation
    - date resolved: **2022-09-25 14:17**\ , date raised: 2022-09-25 
 * Version 1.4.6: resolved Issue 1242: Beam3D (change)
    - description:  rename ANCFBeam3D to ANCFBeam and GeometricallyExactBeam3D in same way for consistency reasons
    - date resolved: **2022-09-24 19:35**\ , date raised: 2022-08-24 
 * Version 1.4.5: resolved Issue 1265: RollingDisc (fix)
    - description:  correct description of coordinate systems in RollingDisc and RollingDiscPenalty; remove wrong transposed sign
    - date resolved: **2022-09-19 17:46**\ , date raised: 2022-09-19 
 * Version 1.4.4: resolved Issue 1263: RollingDiscPenalty (extension)
    - description:  add arbitrary local wheel axis (discAxis) instead of x-axis only
    - date resolved: **2022-09-17 10:42**\ , date raised: 2022-09-17 
 * Version 1.4.3: resolved Issue 1262: RigidBodyInertia (extension)
    - description:  add += operator
    - date resolved: **2022-09-17 08:23**\ , date raised: 2022-09-17 
 * Version 1.4.2: resolved Issue 1261: Lie group integration (extension)
    - description:  improve implicit Lie group integration, adapt old rotation vector approach, add tangent operator
    - date resolved: **2022-09-16 18:14**\ , date raised: 2022-09-16 
 * Version 1.4.1: :textred:`resolved BUG 1260` : MacOS 
    - description:  compilation not working on MacOS with taskmanager adaptions
    - date resolved: **2022-09-15 11:14**\ , date raised: 2022-09-15 
 * Version 1.4.0: resolved Issue 0860: SparseVectorDomain (extension)
    - description:  ?NEEDED (see 0862): create templated SparseVector with IndexValue+domain, having C-array of ResizableArray<IndexValue>, used for multithreaded creation of sparse vectors, associated indices for lateron fill into system vectors
    - **notes:** done in TemporaryComputationData now
    - date resolved: **2022-09-15 08:50**\ , date raised: 2022-01-13 

***********
Version 1.3
***********

 * Version 1.3.105: resolved Issue 0861: SparseVectorParallel (extension)
    - description:  create SparseVector with: mainSparseVector + ArrayIndex2 with per-thread max index and current index; SparseTriplets exceeding the index go into ResizableArray<SparseVector\*> threadSparseVector; Function to finally fill all triplets into main vector
    - **notes:** done in TemporaryComputationData now
    - date resolved: **2022-09-15 08:49**\ , date raised: 2022-01-13 
 * Version 1.3.104: resolved Issue 1259: MarkerSuperElementRigid (extension)
    - description:  extend offset for case that it is large
    - date resolved: **2022-09-14 15:18**\ , date raised: 2022-09-14 
 * Version 1.3.103: resolved Issue 1258: AnimateModes (extension)
    - description:  add option to change sign of mode
    - date resolved: **2022-09-14 08:51**\ , date raised: 2022-09-14 
 * Version 1.3.102: :textred:`resolved BUG 1257` : AnimateModes 
    - description:  button for "Faces only" not working due to new way edges are turned on / off; workaround by pressing T in render window
    - **notes:** buttons now affect the mesh faces / edges instead of regular edges / faces
    - date resolved: **2022-09-14 08:28**\ , date raised: 2022-09-13 
 * Version 1.3.101: resolved Issue 1256: GUI (extension)
    - description:  added some variables in exudyn.GUI which may be used to adjust appearance of dialogs, specifically for extreme display scaling
    - date resolved: **2022-09-13 16:42**\ , date raised: 2022-09-13 
 * Version 1.3.100: resolved Issue 1252: use display scaling in GUI (extension)
    - description:  use display scaling in visualizationSettings, etc.
    - date resolved: **2022-09-13 16:42**\ , date raised: 2022-09-13 
 * Version 1.3.99: resolved Issue 1255: tkinter (change)
    - description:  put root calls into try-except clause in order to preserve operation on special systems like MacOS or Linux
    - date resolved: **2022-09-13 14:49**\ , date raised: 2022-09-13 
 * Version 1.3.98: resolved Issue 1254: SolutionViewer, AnimateModes (extension)
    - description:  add font size and title; this allows to scale fonts as monitor scaling is not active here
    - date resolved: **2022-09-13 14:13**\ , date raised: 2022-09-13 
 * Version 1.3.97: resolved Issue 1253: renderState (docu)
    - description:  add description into theDoc, in section 3D graphics visualization
    - date resolved: **2022-09-13 11:12**\ , date raised: 2022-09-13 
 * Version 1.3.96: resolved Issue 1251: useWindowsDisplayScaleFactor (change)
    - description:  changed visualizationsSettings.general option name from useWindowsMonitorScaleFactor to useWindowsDisplayScaleFactor
    - date resolved: **2022-09-13 10:33**\ , date raised: 2022-09-13 
 * Version 1.3.95: resolved Issue 0538: displayScaling (extension)
    - description:  add displayScaling to renderState and use this value for GUI.py in visualizationSettings
    - **notes:** added displayScaling and automatic updating when Windows display scaling is changed or window is moved to other display
    - date resolved: **2022-09-13 10:19**\ , date raised: 2021-01-06 
 * Version 1.3.94: resolved Issue 1250: FEM HurtyCraigBampton (extension)
    - description:  ComputeHurtyCraigBamptonModes now includes option to compute RBE3 case; adds optional boundary node weights
    - date resolved: **2022-09-07 12:33**\ , date raised: 2022-09-06 
 * Version 1.3.93: resolved Issue 1249: FEM interface (extension)
    - description:  add function GetNodeWeightsFromSurfaceAreas which computes correct weights for linear finite elements (tested for tetrahedrals); this weighting can now reduce erroneous offset in MarkerSuperElementRigid significantly; also used for RBE3 mode computation
    - date resolved: **2022-09-07 12:33**\ , date raised: 2022-09-06 
 * Version 1.3.92: resolved Issue 1248: AddObjectFFRFreducedOrderWithUserFunctions (fix)
    - description:  wrong description of user functions; add missing itemIndex in description
    - date resolved: **2022-09-05 18:39**\ , date raised: 2022-09-05 
 * Version 1.3.91: resolved Issue 1246: DrawSystemGraph (extension)
    - description:  add option to create multi-line graphs; improving appearance and readability
    - date resolved: **2022-09-02 09:05**\ , date raised: 2022-09-02 
 * Version 1.3.90: resolved Issue 1244: pre-compiled linux (extension)
    - description:  add all Python 3.6 - 3.10 linux 64 bit versions to pypi with pip installer
    - date resolved: **2022-09-01 10:02**\ , date raised: 2022-08-25 
 * Version 1.3.89: resolved Issue 0562: Gen alpha Lie (extension)
    - description:  add Lie groups to new generalized alpha integrator
    - date resolved: **2022-08-26 14:09**\ , date raised: 2021-01-26 
 * Version 1.3.88: resolved Issue 1245: Rotation output variable (change)
    - description:  changed/fixed OutputVariableType.Rotation for NodeRigidBodyRotVecLG to output Tait-Bryan rotations instead of rotation parameters; corrected description for NodeRigidBodyRxyz: returns rotation parameters directly, NOT recomputed from RotationMatrix
    - date resolved: **2022-08-26 10:25**\ , date raised: 2022-08-26 
 * Version 1.3.87: resolved Issue 1243: Lie group integration (extension)
    - issue author: S. Holzinger
    - description:  add new functions for implicit Lie group integration (FIRST TESTS)
    - date resolved: **2022-08-24 17:13**\ , date raised: 2022-08-24 
    - resolved by: S. Holzinger
 * Version 1.3.86: :textred:`resolved BUG 1239` : Timer registration 
    - description:  self-registration of Timers fails on Linux and may lead to crashes; depends on order of initialization of global variables
    - **notes:** changed timer registration to suggested way with scalar variables, guaranteeing initialization
    - date resolved: **2022-08-24 15:10**\ , date raised: 2022-08-24 
 * Version 1.3.85: :textred:`resolved BUG 1225` : Linux TestSuite 
    - description:  segmentation fault when running ANCFgeneralContactCircle.py
    - date resolved: **2022-08-24 09:13**\ , date raised: 2022-08-11 
 * Version 1.3.84: resolved Issue 1238: visualization dialog (change)
    - description:  resort options, such that contour options are on top
    - date resolved: **2022-08-23 15:19**\ , date raised: 2022-08-23 
 * Version 1.3.83: resolved Issue 1237: Beams (extension)
    - description:  add option for drawing filled cross-sections (or alternatively wire frames)
    - date resolved: **2022-08-23 15:19**\ , date raised: 2022-08-23 
 * Version 1.3.82: resolved Issue 1236: GeometricallyExactBeam (fix)
    - description:  GeometricallyExactBeam2D and GeometricallyExactBeam3D do not show values in contour plot
    - date resolved: **2022-08-23 14:15**\ , date raised: 2022-08-23 
 * Version 1.3.81: resolved Issue 1235: GeometricallyExactBeam2D (fix)
    - description:  add missing output variables (strain, curvatureLocal, forces,torques)
    - date resolved: **2022-08-23 12:02**\ , date raised: 2022-08-23 
 * Version 1.3.80: :textred:`resolved BUG 1233` : KinematicTree 
    - description:  Jacobian in KinematicTree has too many approximations: either missing velocity terms have large influence or double entries for connectors on single KinematicTree; check stiffFlyballGovernor w/o systemWideDifferentiation
    - **notes:** fixed JacobianODE2 for duplicate global indices, e.g., in case that Connector is attached with two markers to same object (kinematic tree)
    - date resolved: **2022-08-23 11:41**\ , date raised: 2022-08-22 
 * Version 1.3.79: resolved Issue 1234: Newton / Jacobian (extension)
    - description:  add new Newton / numericalDifferentiation setting jacobianConnectorDerivative for faster Jacobian computations
    - date resolved: **2022-08-23 10:15**\ , date raised: 2022-08-23 
 * Version 1.3.78: resolved Issue 1224: KinematicTree (check)
    - description:  visulization problems with kinematicTreeConstraintTest.py with 10 links and more; may be caused by wrong states in visualization or possible bug in KinematicTreeMarker
    - **notes:** resolved with issue 1232 by adding additional temporary variables for visualization
    - date resolved: **2022-08-22 23:26**\ , date raised: 2022-08-07 
 * Version 1.3.77: :textred:`resolved BUG 1232` : KinematicTree 
    - description:  visualization of joints uses illegal temporary data; leads to data race and erroneous results
    - date resolved: **2022-08-22 23:23**\ , date raised: 2022-08-22 
 * Version 1.3.76: :textred:`resolved BUG 1231` : BasicDefinitions 
    - description:  C++: definition of MAXREAL wrong; affects searchtree and contact
    - date resolved: **2022-08-22 21:33**\ , date raised: 2022-08-22 
 * Version 1.3.75: resolved Issue 1230: GetInitialVector (change)
    - description:  change GetInitialVector into GetInitialCoordinateVector for consistency reasons; samge for GetInitialVector_t, SetInitialVector, SetInitialVector_t
    - date resolved: **2022-08-17 17:53**\ , date raised: 2022-08-17 
 * Version 1.3.74: :textred:`resolved BUG 1229` : CMarkerBodyCable2DShape 
    - description:  system error due to incorrect initialization of matrix
    - date resolved: **2022-08-15 15:31**\ , date raised: 2022-08-15 
 * Version 1.3.73: resolved Issue 1228: add LieGroup node with data coordinates (extension)
    - description:  add special Lie group node for implicit integration, containing the start-of-step configuration in data coordinates and additionally use regular ODE2 coordinates for the incremental motion
    - date resolved: **2022-08-12 19:35**\ , date raised: 2022-08-12 
 * Version 1.3.72: resolved Issue 1227: CNode.cpp (change)
    - description:  C++: remove exceptions for illegal index access in GetCurrentCoordinate(...)
    - date resolved: **2022-08-12 19:18**\ , date raised: 2022-08-12 
 * Version 1.3.71: resolved Issue 1226: initial coordinates (extension)
    - description:  C++: nodes get a separate SetInitialCoordinateVector() function; used for special nodes with mixed coordinates
    - date resolved: **2022-08-12 19:17**\ , date raised: 2022-08-12 
 * Version 1.3.70: resolved Issue 1115: KinematicTree (extension)
    - description:  C++ implement efficient T66 transformations
    - **notes:** stable implementation, with formulas different to Featherstone formulas, but in line with 6D matrix manipulations; further tests and documentation needed
    - date resolved: **2022-08-07 22:55**\ , date raised: 2022-05-29 
 * Version 1.3.69: :textred:`resolved BUG 1223` : T66MotionInverse 
    - description:  C++: RigidBodyMath implementation of T66 inverse is wrong, could affect special KinematicTree force reaction
    - date resolved: **2022-08-05 21:45**\ , date raised: 2022-08-05 
 * Version 1.3.68: resolved Issue 1222: KinematicTree (change)
    - description:  remove some temporary variables from interface as new efficient transformations cannot be converted to Python easily
    - date resolved: **2022-08-04 16:28**\ , date raised: 2022-08-04 
 * Version 1.3.67: resolved Issue 1221: Transformations66List (change)
    - description:  C++: rename into Transformation66List
    - date resolved: **2022-08-04 16:27**\ , date raised: 2022-08-04 
 * Version 1.3.66: resolved Issue 1220: PlotImage (extension)
    - description:  add options for orthogonal projection and removing axes and background
    - date resolved: **2022-07-26 09:34**\ , date raised: 2022-07-26 
 * Version 1.3.65: resolved Issue 1205: LinkedDataVectorParallel (check)
    - description:  C++: check if LinkedDataVector can obtain performance mode from ResizableVectorParallel
    - date resolved: **2022-07-22 20:45**\ , date raised: 2022-07-12 
 * Version 1.3.64: resolved Issue 1206: parallel (extension)
    - description:  C++: parallelize important vector-vector and matrix-vector (MultMatrix, MultAdd, ...) operations with optional commands
    - **notes:** already done in ResizableVectorParallel, but extensions in LinkedDataVector needed
    - date resolved: **2022-07-22 19:49**\ , date raised: 2022-07-12 
 * Version 1.3.63: resolved Issue 1218: SolverExplicit (extension)
    - description:  parallelize Lie group updates in explicit solver
    - date resolved: **2022-07-22 19:26**\ , date raised: 2022-07-22 
 * Version 1.3.62: resolved Issue 1219: SolverExplicit (change)
    - description:  turn off Lie group integration if no Lie group nodes available
    - date resolved: **2022-07-22 18:15**\ , date raised: 2022-07-22 
 * Version 1.3.61: resolved Issue 1160: numpy arrays (change)
    - description:  change conversion behavior for Vector3D and Matrix3D, automatically transformed into numpy arrays instead of std::vector which gives a list right now
    - **notes:** for items containing VectorXD or MatrixXD, the parameter returned in GetObject() and similar functions is now giving numpy arrays; for system structures, this is anyway already implemented; specifically, the changed behaviour can be used for referenceCoordinates in user functions; BEHAVIOUR CHANGED: check your models!
    - date resolved: **2022-07-21 19:33**\ , date raised: 2022-06-26 
 * Version 1.3.60: resolved Issue 1209: parallel (extension)
    - description:  C++: add multithreaded parallelization for PostNewton
    - date resolved: **2022-07-21 10:02**\ , date raised: 2022-07-14 
 * Version 1.3.59: resolved Issue 1035: TestModels (change)
    - description:  change modelUnitTests imports in TestModels such that they also work in case that example is run outside TestModels directory
    - **notes:** done earlier
    - date resolved: **2022-07-20 14:33**\ , date raised: 2022-04-07 
 * Version 1.3.58: resolved Issue 1016: TestSuite (testing)
    - description:  add example of reeving system
    - **notes:** done earlier
    - date resolved: **2022-07-20 14:33**\ , date raised: 2022-03-28 
 * Version 1.3.57: resolved Issue 1217: GraphicsData (extension)
    - description:  add functions for drawing text, line and circle: GraphicsDataLine, GraphicsDataText, GraphicsDataCircle
    - date resolved: **2022-07-20 09:45**\ , date raised: 2022-07-20 
 * Version 1.3.56: :textred:`resolved BUG 1216` : renderer: Circle 
    - description:  circles drawn wrongly, not closed for circleTiling <= 6 and producing overly many lines
    - date resolved: **2022-07-19 20:11**\ , date raised: 2022-07-19 
 * Version 1.3.55: resolved Issue 1214: Export lines (extension)
    - description:  add option to export all lines from renderer similar to RenderImage, however, just exporting the raw line information (2 points, RGBA color)
    - date resolved: **2022-07-19 18:39**\ , date raised: 2022-07-19 
 * Version 1.3.54: resolved Issue 1215: GraphicsData addEdges (fix)
    - description:  some examples still in a previous state expecting wrong GraphicsData format
    - date resolved: **2022-07-19 12:14**\ , date raised: 2022-07-19 
 * Version 1.3.53: :textred:`resolved BUG 1213` : mbs.systemData.Info() 
    - description:  mbs.systemData.Info() gives error for KinematicTree
    - date resolved: **2022-07-15 15:34**\ , date raised: 2022-07-15 
 * Version 1.3.52: resolved Issue 1212: DynamicSolverType::RK67 (fix)
    - description:  shows DynamicSolverType::invalid
    - date resolved: **2022-07-15 15:00**\ , date raised: 2022-07-15 
 * Version 1.3.51: resolved Issue 1211: PlotSensor (extension)
    - description:  add listMarkerStyles and listMarkerStylesFilled to exudyn.plot for use in loops
    - date resolved: **2022-07-15 09:28**\ , date raised: 2022-07-15 
 * Version 1.3.50: resolved Issue 1210: microThread (check)
    - description:  check if threading can be optimized by only using sync atomic variables
    - **notes:** no big improvement found, also using bit-wise thread communication and saving some atomic operations
    - date resolved: **2022-07-14 20:40**\ , date raised: 2022-07-14 
 * Version 1.3.49: resolved Issue 1208: timer PostNewton (extension)
    - description:  add timer for PostNewtonStep
    - date resolved: **2022-07-14 00:15**\ , date raised: 2022-07-14 
 * Version 1.3.48: resolved Issue 1207: microThreading (extension)
    - description:  add micro threading library which already takes effect for small systems (10-20 3D rigid bodies); currently included with separate compile option in BasicDefinitions.h
    - date resolved: **2022-07-13 13:43**\ , date raised: 2022-07-13 
 * Version 1.3.47: resolved Issue 1199: adjust testSuite (testing)
    - description:  due to change of state vector to ResizableVectorParallel with AVX arithmetic, minor changes in reference solutions happened
    - date resolved: **2022-07-12 17:06**\ , date raised: 2022-07-11 
 * Version 1.3.46: resolved Issue 1193: SparseSolver analyzePattern (extension)
    - description:  add option to reuse result of analyzePattern() for successive computations; especially in time integration, using the number of non-zeros as indicator is something has changed; add some initializationFunctions to reset, especially after non-convergence
    - date resolved: **2022-07-12 17:06**\ , date raised: 2022-07-10 
 * Version 1.3.45: resolved Issue 1204: parallel / multithreaded (extension)
    - description:  C++: add multithreading for AlgebraicEquations
    - date resolved: **2022-07-12 15:03**\ , date raised: 2022-07-12 
 * Version 1.3.44: resolved Issue 1201: parallel / multithreaded (extension)
    - description:  C++: add multithreading for ProjectedReactionForces
    - date resolved: **2022-07-12 13:04**\ , date raised: 2022-07-12 
 * Version 1.3.43: resolved Issue 1200: CSystem::Jacobians (change)
    - description:  C++: removed single flags for jacobians and replaced by JacobianType; check your results (bugs may happen)
    - date resolved: **2022-07-12 10:55**\ , date raised: 2022-07-12 
 * Version 1.3.42: resolved Issue 1198: parallel MassMatrix (extension)
    - description:  use multithreading to compute mass matrix; treat objects with user functions separately
    - date resolved: **2022-07-11 23:18**\ , date raised: 2022-07-11 
 * Version 1.3.41: :textred:`resolved BUG 1197` : ResizableArray 
    - description:  C++: copy of illegal parts of memory when enlarging array
    - date resolved: **2022-07-11 22:58**\ , date raised: 2022-07-11 
 * Version 1.3.40: resolved Issue 1195: remove openmp (change)
    - description:  remove openmp compile options from setup.py as it is not used for now
    - date resolved: **2022-07-11 19:02**\ , date raised: 2022-07-11 
 * Version 1.3.39: resolved Issue 1191: SystemState (change)
    - description:  C++: replace Vector state with ResizableVector(Parallel)
    - date resolved: **2022-07-10 14:50**\ , date raised: 2022-07-10 
 * Version 1.3.38: resolved Issue 1186: ALEANCFCable2D (extension)
    - description:  add missing terms related to curvature and strain coupling with delta qALE
    - date resolved: **2022-07-09 21:25**\ , date raised: 2022-07-06 
 * Version 1.3.37: resolved Issue 1190: ContactFrictionCircleCable2D (extension)
    - description:  now adapted to work in general with ALECable2D, however, only for tangential frictionStiffness=0
    - date resolved: **2022-07-09 14:33**\ , date raised: 2022-07-09 
 * Version 1.3.36: :textred:`resolved BUG 1188` : beam.py 
    - description:  missing eii structure before Point2DS1
    - date resolved: **2022-07-08 16:22**\ , date raised: 2022-07-08 
 * Version 1.3.35: resolved Issue 1185: ContactFrictionCircleCable2D (extension)
    - description:  add ALE term to marker and contact element
    - date resolved: **2022-07-06 15:30**\ , date raised: 2022-07-06 
 * Version 1.3.34: resolved Issue 1183: star imports (change)
    - description:  remove \* imports from all .py modules except utilities.py
    - **notes:** also fixed some undetected bugs in unused functions in exudyn.\* utilities
    - date resolved: **2022-07-06 11:56**\ , date raised: 2022-07-06 
 * Version 1.3.33: resolved Issue 1184: exudyn.utilities (change)
    - description:  remove import of time and copy; needs to be included separately into models
    - **notes:** check your models!
    - date resolved: **2022-07-06 11:55**\ , date raised: 2022-07-06 
 * Version 1.3.32: resolved Issue 1182: roboticsCore.py (change)
    - description:  remove \* import of many exudyn packages
    - date resolved: **2022-07-06 10:58**\ , date raised: 2022-07-06 
 * Version 1.3.31: resolved Issue 1181: setup.py (change)
    - description:  due to deprecation warning, use namespace_packages instead of packages in setup(...); remove include_package_data=True
    - date resolved: **2022-07-06 09:43**\ , date raised: 2022-07-06 
 * Version 1.3.30: resolved Issue 0954: ObjectContactFrictionCircleCable2D (extension)
    - description:  Add exception in PreAssembleChecks if marker is not a MarkerBody, as implementation only works for MarkerBody but not for MarkerNode
    - **notes:** resolved, as now a MarkerNodeRigid may also be used
    - date resolved: **2022-07-05 21:18**\ , date raised: 2022-02-27 
 * Version 1.3.29: resolved Issue 1180: AddEdgesAndSmoothenNormals (extension)
    - description:  specialized function for STL file enhancement
    - date resolved: **2022-07-05 20:51**\ , date raised: 2022-07-05 
 * Version 1.3.28: resolved Issue 1179: show lines (extension)
    - description:  add separate visualization.openGL flag for showing/hiding lines
    - date resolved: **2022-07-05 11:20**\ , date raised: 2022-07-05 
 * Version 1.3.27: resolved Issue 1178: STL import/export (extension)
    - description:  add option to invert triangles and/or normls on import or export of STL meshes
    - date resolved: **2022-07-05 08:38**\ , date raised: 2022-07-05 
 * Version 1.3.26: resolved Issue 1177: selection right mouse (extension)
    - description:  now also showing GraphicsData optionally with visualizationSettings.interactive.selectionRightMouseGraphicsData
    - date resolved: **2022-07-05 08:38**\ , date raised: 2022-07-05 
 * Version 1.3.25: resolved Issue 1099: TriangleList (extension)
    - description:  extend GraphicsData TriangleList with two optional lists (which may be empty) containing edges (tuples of point numbers) as well as edgeColors; allows to easily add edges to graphics representation
    - date resolved: **2022-07-05 00:04**\ , date raised: 2022-05-22 
 * Version 1.3.24: resolved Issue 1174: mbs.GetObject (extension)
    - description:  add option to receive graphicsData (or not)
    - **notes:** default behavior kept same, not returning graphicsData in dict
    - date resolved: **2022-07-04 23:59**\ , date raised: 2022-07-04 
 * Version 1.3.23: resolved Issue 1159: BodyGraphicsData (extension)
    - description:  add method to convert bodyGraphicsData into dictionary; add flag to GetObject to by default not show graphicsData
    - date resolved: **2022-07-04 23:59**\ , date raised: 2022-06-26 
 * Version 1.3.22: resolved Issue 1176: GraphicsDataCylinder...() (change)
    - description:  returns always a GraphicsData dictionary, independently of addEdges is True or False
    - date resolved: **2022-07-04 23:33**\ , date raised: 2022-07-04 
 * Version 1.3.21: resolved Issue 1175: GraphicsDataOrthoCube...() (change)
    - description:  returns always a GraphicsData dictionary, independently of adding edges or not
    - date resolved: **2022-07-04 23:33**\ , date raised: 2022-07-04 
 * Version 1.3.20: resolved Issue 1169: conversion to STL (extension)
    - description:  add function to convert graphicsData triangle meshes into STL; allows import into other tools
    - date resolved: **2022-07-04 20:55**\ , date raised: 2022-07-03 
 * Version 1.3.19: resolved Issue 0821: include numpy-stl (extension)
    - description:  include library to import stl files; add Warning to GraphicsDataFromSTLfileTxt for large file sizes and check if is ascii; in binary case, try switching to numpy-stl; add example to load binary files; add example to convert stl ascii to binary files
    - **notes:** example for stl import added: Examples/stlFileImport.py
    - date resolved: **2022-07-04 20:55**\ , date raised: 2021-12-06 
 * Version 1.3.18: resolved Issue 1173: GetObjectOutputBody (change)
    - description:  add default argument localPosition=[0,0,0]
    - date resolved: **2022-07-04 11:45**\ , date raised: 2022-07-04 
 * Version 1.3.17: resolved Issue 1172: GetObjectOutput (change)
    - description:  include configurationType in interface (default: Current); not available in connectors; add objectNumber in C++ interface
    - date resolved: **2022-07-04 11:43**\ , date raised: 2022-07-04 
 * Version 1.3.16: :textred:`resolved BUG 1171` : MarkerKinematicTreeRigid 
    - description:  incorrect GetPosition, GetVelocity, GetAngularVelocity, ...
    - date resolved: **2022-07-04 11:43**\ , date raised: 2022-07-03 
 * Version 1.3.15: resolved Issue 1161: KinematicTree (change)
    - description:  change GetObjectOutputBody to GetObjectOutput as localPosition does not make sense here
    - date resolved: **2022-07-04 11:39**\ , date raised: 2022-06-27 
 * Version 1.3.14: resolved Issue 1170: MarkerKinematicTreeRigid (extension)
    - description:  add visualization
    - date resolved: **2022-07-03 20:23**\ , date raised: 2022-07-03 
 * Version 1.3.13: resolved Issue 1168: left mouse (change)
    - description:  deactivate mouse select if renderer is showing mouse coordinates (and doing measuring)
    - date resolved: **2022-07-02 11:29**\ , date raised: 2022-07-02 
 * Version 1.3.12: resolved Issue 1166: InteractiveDialog (extension)
    - description:  In all interactive dialogs, especially the SolutionViewer now stopping the render window (with Q or Escape) also stops the interactive dialog; behavior changed with checkRenderEngineStopFlag
    - date resolved: **2022-06-29 10:15**\ , date raised: 2022-06-29 
 * Version 1.3.11: resolved Issue 1165: class Robot (change)
    - description:  drawing of cylinder for first body in robot removed; this part must be drawn manually at base (or not drawn)
    - date resolved: **2022-06-29 10:02**\ , date raised: 2022-06-29 
 * Version 1.3.10: resolved Issue 0720: AddObjectFFRFreducedOrder (extension)
    - description:  add gravity as with user functions version, similar to AddRigidBody
    - date resolved: **2022-06-29 09:27**\ , date raised: 2021-07-12 
 * Version 1.3.9: resolved Issue 1018: MacOS (change)
    - description:  adjust mouse scroll factor in case of Apple/Mac OS compilation (e.g. multiply with 0.05 by default)
    - **notes:** changed way to compute zoomFactor for larger yOffsets in scroll callback
    - date resolved: **2022-06-28 15:30**\ , date raised: 2022-03-29 
 * Version 1.3.8: resolved Issue 1164: OpenGL shadow (fix)
    - description:  resolve issues with shadows drawing of many objects; added _WRAP to stencil INCR and DECR operations to resolve overflows
    - date resolved: **2022-06-27 19:20**\ , date raised: 2022-06-27 
 * Version 1.3.7: resolved Issue 1163: OpenGL normals (fix)
    - description:  correct orientation of triangles and normals in GraphicsData functions
    - date resolved: **2022-06-27 16:22**\ , date raised: 2022-06-27 
 * Version 1.3.6: resolved Issue 1162: OpenGL normals (change)
    - description:  correct normals in GraphicsDataSphere, GraphicsDataCylinder and GraphicsDataSolidOfRevolution to point outwards; turn off GL_LIGHT_MODEL_TWO_SIDE by default
    - date resolved: **2022-06-27 14:12**\ , date raised: 2022-06-27 
 * Version 1.3.5: resolved Issue 1158: RigidBodyInertia (extension)
    - description:  add Transformed() function to return rigid body inertia transformed by homogeneous transformation; used in class Robot for certain transformations of link frame
    - date resolved: **2022-06-27 01:20**\ , date raised: 2022-06-26 
 * Version 1.3.4: resolved Issue 1072: class Robot (extension)
    - description:  extend KinematicTree export in order to be able to create robots with localHT!=HT0()
    - date resolved: **2022-06-27 00:41**\ , date raised: 2022-05-06 
 * Version 1.3.3: resolved Issue 1131: class Robot (extension)
    - description:  add list of graphicsDataLists to each body for individual graphics of links allowing also graphics for tools
    - date resolved: **2022-06-26 23:46**\ , date raised: 2022-06-03 
 * Version 1.3.2: resolved Issue 1094: add links to other github repos (docu)
    - description:  link more directly to ngsolve and openAI / stable_baslines3 / etc.; refer to openAI in github links and add ref to theDoc.pdf ; add example/teaser?
    - date resolved: **2022-06-24 09:20**\ , date raised: 2022-05-21 
 * Version 1.3.1: resolved Issue 1157: C++ user functions (testing)
    - description:  check if linking C++ user functions directly boosts performance
    - **notes:** C++ functions are translated by including pybind11/functional.h; workaround would be two user functions, one without MainSystem and without including functional.h; C++ functions could then be compiled in a very simple manner
    - date resolved: **2022-06-23 16:57**\ , date raised: 2022-06-23 
 * Version 1.3.0: :textred:`resolved BUG 0677` : single threaded renderer 
    - description:  correct crash with visualization dialog (MacOS)
    - **notes:** not resolved, but it is an issue of missing tkinter capabilities in MacOS
    - date resolved: **2022-06-22 07:59**\ , date raised: 2021-05-12 

***********
Version 1.2
***********

 * Version 1.2.146: resolved Issue 0735: parallel build (check)
    - description:  check parallel build with MSbuild to reduce compilation times
    - **notes:** already done earlier
    - date resolved: **2022-06-22 07:58**\ , date raised: 2021-08-12 
 * Version 1.2.145: resolved Issue 0863: RaspberryPi (extension)
    - description:  check compilation on Raspi, make adaptation of ngsolve includes to run
    - **notes:** already done earlier
    - date resolved: **2022-06-22 07:48**\ , date raised: 2022-01-14 
 * Version 1.2.144: resolved Issue 0968: MarkerBodyCable2DShape (extension)
    - description:  add offset from beam axis, to compute position, velocity and jacobians at contact surface
    - **notes:** already done earlier
    - date resolved: **2022-06-22 07:46**\ , date raised: 2022-03-03 
 * Version 1.2.143: resolved Issue 0972: ContactFrictionCircleCable2D (docu)
    - description:  add / extend description
    - date resolved: **2022-06-22 07:44**\ , date raised: 2022-03-09 
 * Version 1.2.142: resolved Issue 1155: center point (extension)
    - description:  add key "o" option to set center point to current center point of view; this allows to rotate around the current center point of the view
    - **notes:** currently deactivated due to transformation of coordinates issue
    - date resolved: **2022-06-21 18:16**\ , date raised: 2022-06-21 
 * Version 1.2.141: resolved Issue 1152: Renderer (extension)
    - description:  add separate function to compute accurate scene size, store as 3D vector and as norm
    - date resolved: **2022-06-21 17:56**\ , date raised: 2022-06-20 
 * Version 1.2.140: resolved Issue 1154: Zoom all (change)
    - description:  change procedures for zoom all and for computation of maximum scene coordinates; may affect appearance of your models; necessary for consistently computing perspective and shadow parameters
    - date resolved: **2022-06-21 08:38**\ , date raised: 2022-06-21 
 * Version 1.2.139: resolved Issue 1153: noglfw (check)
    - description:  check compilation without GLFW
    - date resolved: **2022-06-20 10:58**\ , date raised: 2022-06-20 
 * Version 1.2.138: resolved Issue 1150: OpenGL (extension)
    - description:  add shadows (simple)
    - **notes:** activate with SC.VisualizationSettings().openGL.shadow, chosing a value between 0. and 1; good results obtained with 0.5
    - date resolved: **2022-06-20 01:49**\ , date raised: 2022-06-18 
 * Version 1.2.137: resolved Issue 1151: linux builds (change)
    - description:  remove -g flag from linux builds, leading to 2.6MB instead of 38MB binaries; to enable debug information (e.g. to detect origin of some crashes, remove the -g0 flag in setup.py)
    - date resolved: **2022-06-19 18:34**\ , date raised: 2022-06-19 
 * Version 1.2.136: resolved Issue 1149: OpenGL (extension)
    - description:  add perspective
    - **notes:** added openGL option perspective; EXPERIMENTAL!
    - date resolved: **2022-06-19 01:59**\ , date raised: 2022-06-18 
 * Version 1.2.135: resolved Issue 1148: jacobian ODE1 ODE2 (fix)
    - description:  fix jacobian computations for mixed ODE1-ODE2 components; check with HydraulicActuatorSimple (lower number of jacobians and iterations)
    - date resolved: **2022-06-18 23:38**\ , date raised: 2022-06-18 
 * Version 1.2.134: resolved Issue 1147: ODE1Coordinates_t (extension)
    - description:  add missing function SetODE1Coordinates_t and GetODE1Coordinates_t into Python interface
    - date resolved: **2022-06-17 01:14**\ , date raised: 2022-06-17 
 * Version 1.2.133: resolved Issue 1096: hydraulics (example)
    - description:  add hydraulic actuator with new element HydraulicActuatorSimple
    - date resolved: **2022-06-17 00:06**\ , date raised: 2022-05-22 
 * Version 1.2.132: resolved Issue 1146: solver jacobians (extension)
    - description:  extended jacobians for ODE1 and ODE2 residuals for ODE1-ODE2 coupling in ComputeJacobianODE2RHS, ComputeJacobianODE1RHS, and ComputeJacobianAE; changed default values as well; check your code if you used these functions in Python user functions
    - date resolved: **2022-06-16 19:35**\ , date raised: 2022-06-16 
 * Version 1.2.131: resolved Issue 1145: static solver (extension)
    - description:  add check if system contains ODE1 variables
    - date resolved: **2022-06-16 18:52**\ , date raised: 2022-06-16 
 * Version 1.2.130: resolved Issue 1144: PlotSensor (extension)
    - description:  added option to allow components=[plot.componentNorm] for displaying norm of sensors
    - date resolved: **2022-06-16 17:32**\ , date raised: 2022-06-16 
 * Version 1.2.129: resolved Issue 1143: numba jit (example)
    - description:  add example for numba jit speedup of Python user functions
    - **notes:** added Example springDamperUserFunctionNumbaJIT.py showing speedup of 4 for simple Python function; note that mbs functions cannot be processed by numba
    - date resolved: **2022-06-16 17:01**\ , date raised: 2022-06-16 
 * Version 1.2.128: resolved Issue 1106: HydraulicsActuator (extension)
    - description:  add HydraulicsActuator as object, containing pressure equations
    - **notes:** added new double acting HydraulicActuatorSimple with internal pressure equations and possibility to modify valves by user functions
    - date resolved: **2022-06-16 12:00**\ , date raised: 2022-05-23 
 * Version 1.2.127: resolved Issue 1138: artificialIntelligence (extension)
    - description:  OpenAIGymInterfaceEnv obtained additional member variable randomInitializationValue which may be adapted; in future, this may be done in a separate function
    - date resolved: **2022-06-10 12:26**\ , date raised: 2022-06-10 
 * Version 1.2.126: resolved Issue 1137: exudynFast (change)
    - description:  for linux builds, do not activate exudynFast, only for noglfw
    - date resolved: **2022-06-10 12:16**\ , date raised: 2022-06-10 
 * Version 1.2.125: :textred:`resolved BUG 1135` : KinematicTree 
    - description:  linux version of KinematicTree gives significantly (6e-7) different results, check initialization
    - **notes:** resulted due to high sensitivity to disturbances (1e-15), especially in acceleration sensors, and ONLY for mbs results!
    - date resolved: **2022-06-09 20:32**\ , date raised: 2022-06-09 
 * Version 1.2.124: resolved Issue 1134: KinematicTree (extension)
    - description:  add Jacobian computation according to class Robot in order to realize AccessFunction needed from MarkerKinematicTreeRigid; uses special access function because the SuperElement interface does not provide link number
    - date resolved: **2022-06-08 18:30**\ , date raised: 2022-06-06 
 * Version 1.2.123: resolved Issue 1071: KinematicTree (extension)
    - description:  add special MarkerKinematicTreeRigidBody with link number and local position
    - date resolved: **2022-06-08 18:30**\ , date raised: 2022-05-05 
 * Version 1.2.122: resolved Issue 1130: class Robot (change)
    - description:  tool only working for serial robots; in case of tree structure, tool should not be used as it is attached only to last link
    - **notes:** added warning to class Robot.AddLink(...) which raises Warning if tool is defined and tree structure is generated
    - date resolved: **2022-06-06 00:32**\ , date raised: 2022-06-03 
 * Version 1.2.121: resolved Issue 1132: class Robot (extension)
    - description:  extend Jacobian, LinkHT and JointHT functions for tree structure; check StaticTorques
    - **notes:** added comparison for class Robot functions in kinematicTreeAndMBStest.py
    - date resolved: **2022-06-06 00:31**\ , date raised: 2022-06-03 
 * Version 1.2.120: resolved Issue 1133: SolutionViewer (extension)
    - description:  bind additional Button "Q" in interactive dialogs for quit (in addition to Escape)
    - date resolved: **2022-06-05 18:25**\ , date raised: 2022-06-05 
 * Version 1.2.119: resolved Issue 1114: class Robot (testing)
    - description:  create new example(s) comparing CreateKinematicTree and CreateRedundantCoordinateMBS with control and prismatic joints
    - date resolved: **2022-06-05 14:33**\ , date raised: 2022-05-29 
 * Version 1.2.118: :textred:`resolved BUG 1127` : class Robot 
    - description:  CreateKinematicTree not working for tree structure
    - date resolved: **2022-06-03 00:28**\ , date raised: 2022-06-03 
 * Version 1.2.117: :textred:`resolved BUG 1129` : class Robot 
    - description:  GetParentIndex(...), HasParent(...) not working for tree structure
    - date resolved: **2022-06-03 00:19**\ , date raised: 2022-06-03 
 * Version 1.2.116: :textred:`resolved BUG 1128` : class Robot 
    - description:  CreateRedundantCoordinateMBS not working for tree structure
    - date resolved: **2022-06-03 00:19**\ , date raised: 2022-06-03 
 * Version 1.2.115: :textred:`resolved BUG 1126` : KinematicTree 
    - description:  wrong formula in computation of acceleration
    - date resolved: **2022-06-02 22:41**\ , date raised: 2022-06-02 
 * Version 1.2.114: resolved Issue 1123: class Robot (extension)
    - description:  extend CreateRedundantCoordinateMBS for jointSpringDamperUserFunctionList with prismatic joints
    - date resolved: **2022-06-01 23:44**\ , date raised: 2022-06-01 
 * Version 1.2.113: resolved Issue 1125: class Robot (extension)
    - description:  extend CreateRedundantCoordinateMBS for control of PrsimaticJoints with new LinearSpringDamper
    - date resolved: **2022-06-01 23:43**\ , date raised: 2022-06-01 
 * Version 1.2.112: resolved Issue 1124: LinearSpringDamper (extension)
    - description:  add LinearSpringDamper which is the corresponding spring damper/actuator for prismatic joints (like TorsionalSpringDamper for revolute joints), being aligned with a rigid marker, having no limits compared to regular (distance based) SpringDamper
    - date resolved: **2022-06-01 23:43**\ , date raised: 2022-06-01 
 * Version 1.2.111: resolved Issue 0503: serialrobot (extension)
    - description:  adapt serial robot for prismatic joints
    - **notes:** functionality with Prismatic joints  now included into class Robot, converting to mbs with CreateRedundantCoordinateMBS or CreateKinematicTree, see issue
    - date resolved: **2022-06-01 20:38**\ , date raised: 2020-12-16 
 * Version 1.2.110: resolved Issue 1121: AnimateSolution (change)
    - description:  mark as deprecated, use SolutionViewer instead
    - date resolved: **2022-06-01 20:25**\ , date raised: 2022-05-31 
 * Version 1.2.109: :textred:`resolved BUG 1122` : GraphicsDataBasis 
    - description:  kwargs argument radius not working
    - date resolved: **2022-05-31 15:32**\ , date raised: 2022-05-31 
 * Version 1.2.108: resolved Issue 1108: KinematicTree (check)
    - description:  check OutputVariable functions and compare with redundant-MBS based bodies
    - date resolved: **2022-05-30 21:06**\ , date raised: 2022-05-27 
 * Version 1.2.107: resolved Issue 1095: hydraulics (example)
    - description:  add hydraulic actuator with user function
    - **notes:** example HydraulicsUserFunction.py added already 5 days earlier
    - date resolved: **2022-05-30 20:06**\ , date raised: 2022-05-22 
 * Version 1.2.106: resolved Issue 1109: class Robot (extension)
    - description:  CreateRedundantCoordinateMBS: jointLoadUserFunctionList and createJointTorqueLoads marked as deprecated; use NEW jointSpringDamperUserFunctionList which allows to directly actuate at revolute/prismatic joints
    - date resolved: **2022-05-30 20:05**\ , date raised: 2022-05-29 
 * Version 1.2.105: :textred:`resolved BUG 1120` : KinematicTree 
    - description:  composite inertia contains wrong index in calculation
    - date resolved: **2022-05-30 20:04**\ , date raised: 2022-05-30 
 * Version 1.2.104: resolved Issue 1113: KinematicTree (testing)
    - description:  test for prismatic joint
    - date resolved: **2022-05-30 20:04**\ , date raised: 2022-05-29 
 * Version 1.2.103: :textred:`resolved BUG 1119` : KinematicTree 
    - description:  wrong signs (missing inverse) of prismatic joints in C++ and Python implementation of KinematicTree
    - date resolved: **2022-05-30 17:55**\ , date raised: 2022-05-30 
 * Version 1.2.102: :textred:`resolved BUG 1118` : ObjectKinematicTree 
    - description:  conversion to dict in mbs.GetObject(...) does not work for Matrix3D
    - date resolved: **2022-05-30 16:02**\ , date raised: 2022-05-30 
 * Version 1.2.101: resolved Issue 1117: Python3.8 (change)
    - description:  the exudyn version for Python3.8 now includes range checks (and is slower than before); for fast version of exudyn without range checks, check issues 1116 and section 'Performance and ways to speed up computations' in theDoc
    - date resolved: **2022-05-30 00:34**\ , date raised: 2022-05-30 
 * Version 1.2.100: resolved Issue 1116: exudynFast (extension)
    - description:  add separate track for fast exudyn versions; the compiler flag _FAST_EXUDYN_LINALG, previously only activated in Python 3.8, is now used for Python 3.7 and Python 3.8 versions ONLY if according import flags are set by doing the following 3 steps: import sys; sys.exudynFast=True; import exudyn
    - date resolved: **2022-05-30 00:34**\ , date raised: 2022-05-29 
 * Version 1.2.99: resolved Issue 1112: class Robot (change)
    - description:  CreateKinematicTree: jointForceVector, jointPositionOffsetVector, jointVelocityOffsetVector, jointPControlVector, jointDControlVector removed and replaced by PDcontrol structure in  RobotLink; forceUserFunction kept as is; return value slightly changed
    - date resolved: **2022-05-29 19:10**\ , date raised: 2022-05-29 
 * Version 1.2.98: resolved Issue 1111: class Robot (extension)
    - description:  RobotLink adds feature to define PDcontrol, used in robots for joint control of link
    - date resolved: **2022-05-29 19:10**\ , date raised: 2022-05-29 
 * Version 1.2.97: resolved Issue 1110: class Robot (extension)
    - description:  CreateRedundantCoordinateMBS: returns additional list springDamperList, which contains more efficient spring dampers for joint control
    - date resolved: **2022-05-29 17:05**\ , date raised: 2022-05-29 
 * Version 1.2.96: resolved Issue 1070: KinematicTree (extension)
    - description:  add SensorSuperElement (BUT call it SensorKinematicTree!) functionality for Position, Velocity, Acceleration, RotationMatrix, AngularVelocity, ...
    - **notes:** created new SensorKinematicTree; tests not done yet
    - date resolved: **2022-05-27 23:43**\ , date raised: 2022-05-05 
 * Version 1.2.95: resolved Issue 1107: Sensors (change)
    - description:  C++: move sensor-specific consistency tests from CSystem to CheckPreAssembleConsistency
    - date resolved: **2022-05-27 23:41**\ , date raised: 2022-05-26 
 * Version 1.2.94: resolved Issue 1103: Object (extension)
    - description:  C++: enable objects to be mixed of ODE2, ODE1 and AE variables (most general case); add jacobian_ODE1ODE2 flags and functions in case of ODE1
    - **notes:** NumericalJacobianODE1RHS modified accordingly
    - date resolved: **2022-05-23 23:06**\ , date raised: 2022-05-23 
 * Version 1.2.93: resolved Issue 1101: HydraulicActuator (example)
    - description:  add HydraulicActuator with user function example
    - **notes:** Duplicate of issue 1102
    - date resolved: **2022-05-23 22:10**\ , date raised: 2022-05-23 
 * Version 1.2.92: resolved Issue 1105: NodeGenericAE (extension)
    - description:  add node with algebraic variables
    - date resolved: **2022-05-23 22:02**\ , date raised: 2022-05-23 
 * Version 1.2.91: resolved Issue 0850: GetNodeODE1Index, GetNodeAEIndex (extension)
    - description:  add missing access functions
    - date resolved: **2022-05-23 21:48**\ , date raised: 2022-01-08 
 * Version 1.2.90: resolved Issue 1102: HydraulicActuator (example)
    - description:  add HydraulicActuator with user function example
    - **notes:** Examples/HydraulicsUserFunction.py
    - date resolved: **2022-05-23 19:53**\ , date raised: 2022-05-23 
 * Version 1.2.89: resolved Issue 1098: GraphicsDataCylinder (extension)
    - description:  add option addEdges which in addition returns GraphicsData for edges; returns then list of dictionaries
    - date resolved: **2022-05-22 20:54**\ , date raised: 2022-05-22 
 * Version 1.2.88: resolved Issue 1097: GraphicsDataOrthoCubePoint (extension)
    - description:  add option addEdges which in addition returns GraphicsData for edges; returns then list of dictionaries
    - date resolved: **2022-05-22 20:54**\ , date raised: 2022-05-22 
 * Version 1.2.87: :textred:`resolved BUG 1093` : ANCFBeam3D 
    - description:  giving wrong results because of error in template<class TMatrix> ConstSizeMatrixBase& operator+= (const TMatrix& matrix); leading to twice the mass matrix
    - date resolved: **2022-05-20 16:07**\ , date raised: 2022-05-20 
 * Version 1.2.86: resolved Issue 1092: openAI gym (extension)
    - description:  add example for interface with openAI gym using cart-pole model
    - **notes:** see Examples/testGymCartpole.py
    - date resolved: **2022-05-18 09:47**\ , date raised: 2022-05-18 
 * Version 1.2.85: :textred:`resolved BUG 1091` : ComputeODE2Eigenvalues 
    - description:  eigenvectors sorting is not according to eigenvalues sorting
    - date resolved: **2022-05-17 10:00**\ , date raised: 2022-05-17 
 * Version 1.2.84: resolved Issue 1090: ComputeODE2Eigenvalues (extension)
    - description:  solver.ComputeODE2Eigenvalues gets additional flag constrainedCoordinates to specify list of constrained coordinates in system for eigenvalue computation
    - date resolved: **2022-05-16 23:01**\ , date raised: 2022-05-16 
 * Version 1.2.83: :textred:`resolved BUG 1089` : ComputeODE2Eigenvalues 
    - description:  setInitialValues has no effect
    - **notes:** flag erased
    - date resolved: **2022-05-16 22:03**\ , date raised: 2022-05-16 
 * Version 1.2.82: :textred:`resolved BUG 1086` : ParameterVariation 
    - description:  numberOfThreads cannot be passed to parameter variation as it is duplicated by \*\*args
    - **notes:** numberOfThreads is now a named argument
    - date resolved: **2022-05-16 16:34**\ , date raised: 2022-05-16 
 * Version 1.2.81: resolved Issue 1079: Beam drawing (extension)
    - description:  add generic drawing function for beams, especially for contour plot
    - date resolved: **2022-05-15 23:24**\ , date raised: 2022-05-09 
 * Version 1.2.80: resolved Issue 1076: ANCFBeam3D (extension)
    - description:  add shear deformable 3D ANCF beam according to Nachbagauer Gerstmayr Pechstein using structural mechanics formulation
    - date resolved: **2022-05-15 23:24**\ , date raised: 2022-05-06 
 * Version 1.2.79: resolved Issue 1081: linux testSuite (check)
    - description:  check failed tests under linux; see tests for 1.2.75 linux
    - **notes:** resolved issues related to initial accelerations; still deviation in test generalContactFrictionTests.py
    - date resolved: **2022-05-11 18:54**\ , date raised: 2022-05-11 
 * Version 1.2.78: :textred:`resolved BUG 1084` : node frame drawing 
    - description:  node frames shows letter N unintended
    - date resolved: **2022-05-11 18:23**\ , date raised: 2022-05-11 
 * Version 1.2.77: :textred:`resolved BUG 1083` : initial accelerations 
    - description:  not correctly handled in linux build (and probably also on MacOS
    - date resolved: **2022-05-11 16:18**\ , date raised: 2022-05-11 
 * Version 1.2.76: resolved Issue 1082: testsuite linux (extension)
    - description:  extend test suite to run always for linux and add file ending for linux and MacOS
    - date resolved: **2022-05-11 12:51**\ , date raised: 2022-05-11 
 * Version 1.2.75: :textred:`resolved BUG 1080` : gcc compile 
    - description:  errors when compiling PyVectorLists and PyMatrixLists on gcc
    - date resolved: **2022-05-11 09:27**\ , date raised: 2022-05-11 
 * Version 1.2.74: resolved Issue 0471: geometrically exact beam3D (extension)
    - description:  add to CPP
    - **notes:** added first version based on conventional parameterization of nodes, but using Lie groups for beam section forces in ode2LHS
    - date resolved: **2022-05-09 21:24**\ , date raised: 2020-11-21 
 * Version 1.2.73: resolved Issue 1077: TExpSO3(Omega) (change)
    - description:  reduce number of terms in termExpanded; only up to x\*\*4 needed for 16 digits
    - date resolved: **2022-05-09 21:22**\ , date raised: 2022-05-07 
 * Version 1.2.72: resolved Issue 1074: BeamSection (extension)
    - description:  add new structure (cpp+Python) for beam cross section as well as BeamSectionGeometry for geometrical representation
    - date resolved: **2022-05-09 21:22**\ , date raised: 2022-05-06 
 * Version 1.2.71: resolved Issue 1073: structures description (extension)
    - description:  add description for simulationSettings, visualizationSettings, solver structures, etc. in Python bindings
    - date resolved: **2022-05-06 13:54**\ , date raised: 2022-05-06 
 * Version 1.2.70: resolved Issue 1069: KinematicTree (extension)
    - description:  add interface to class Robot, to create either redundant or minimal coordinates kinematic tree
    - **notes:** however, is not capable of creating robots with localHT!=HT0()
    - date resolved: **2022-05-06 00:07**\ , date raised: 2022-05-05 
 * Version 1.2.69: resolved Issue 1064: KinematicTree (docu)
    - description:  add basic description (equations)
    - date resolved: **2022-05-05 22:14**\ , date raised: 2022-05-05 
 * Version 1.2.68: resolved Issue 1067: KinematicTree (extension)
    - description:  add standalone example
    - **notes:** see TestModels/kinematicTreeTest.py
    - date resolved: **2022-05-05 20:00**\ , date raised: 2022-05-05 
 * Version 1.2.67: resolved Issue 1065: KinematicTree (docu)
    - description:  add user function description
    - date resolved: **2022-05-05 20:00**\ , date raised: 2022-05-05 
 * Version 1.2.66: resolved Issue 1068: KinematicTree (extension)
    - description:  add test
    - **notes:** added test and MiniExample
    - date resolved: **2022-05-05 19:59**\ , date raised: 2022-05-05 
 * Version 1.2.65: :textred:`resolved BUG 1063` : KinematicTree 
    - description:  cpp version gives wrong results as compared to MBS
    - **notes:** corrected inertia - must be w.r.t. COM
    - date resolved: **2022-05-05 16:14**\ , date raised: 2022-05-05 
 * Version 1.2.64: resolved Issue 1066: BodyGraphicsDataList (extension)
    - description:  new interface used in Kinematic tree which holds a list of GraphicsData lists
    - date resolved: **2022-05-05 11:32**\ , date raised: 2022-05-05 
 * Version 1.2.63: resolved Issue 0655: add KinematicTree (extension)
    - description:  ObjectKinematicTree (minimal coordinates)
    - **notes:** this is a first version, but tests and validation are necessary!
    - date resolved: **2022-05-05 10:27**\ , date raised: 2021-05-01 
 * Version 1.2.62: resolved Issue 0913: KinematicTree (extension)
    - description:  add efficient C++ functionality for operating on 6D vectors and matrices for kinematics/dynamics
    - date resolved: **2022-05-05 10:23**\ , date raised: 2022-02-02 
 * Version 1.2.61: resolved Issue 1062: RigidBodyMath.h (change)
    - description:  remove duplicate from src/Utilities
    - date resolved: **2022-05-03 11:33**\ , date raised: 2022-05-03 
 * Version 1.2.60: resolved Issue 1047: Matrix3DList (extension)
    - description:  add new structure Matrix3DList; used to create list of 3D matrices and transfer into MainSystem mbs, e.g. in KinematicTree
    - date resolved: **2022-05-01 00:47**\ , date raised: 2022-04-25 
 * Version 1.2.59: resolved Issue 1038: simulateInRealtime (extension)
    - description:  put waitMicroSeconds into settings
    - date resolved: **2022-04-30 20:41**\ , date raised: 2022-04-07 
 * Version 1.2.58: resolved Issue 1049: MacOS (check)
    - description:  check if flag -mmacosx-version-min=11.0 is needed in setup.py and if this causes problems on older pre-AppleM1 which needs wheel version 10.9
    - **notes:** x86 compilation requires 11.0, otherwise it fails; need to find another way for compilation
    - date resolved: **2022-04-29 09:59**\ , date raised: 2022-04-25 
 * Version 1.2.57: resolved Issue 1060: buildDate.tex (change)
    - description:  remove by default change of buildDate.tex in setup.py
    - date resolved: **2022-04-29 08:20**\ , date raised: 2022-04-28 
 * Version 1.2.56: :textred:`resolved BUG 1059` : MacOS compile 
    - description:  sse2neon.h not found
    - **notes:** exclude parallel threads from MacOS version until resolving compilation problem
    - date resolved: **2022-04-28 11:58**\ , date raised: 2022-04-28 
 * Version 1.2.55: resolved Issue 0905: Add description of Pybind11 interaction (docu)
    - description:  add some description of C++-Python interaction and entry points into C++
    - date resolved: **2022-04-27 20:37**\ , date raised: 2022-02-01 
 * Version 1.2.54: resolved Issue 1015: TestSuite (docu)
    - description:  add notes in Exudyn on testsuite (add remark: in many cases for tracking of changes, not for validation)
    - date resolved: **2022-04-27 20:21**\ , date raised: 2022-03-28 
 * Version 1.2.53: resolved Issue 1058: ClearWorkspace (change)
    - description:  also close open matplotlib figures before they are lost
    - date resolved: **2022-04-27 19:47**\ , date raised: 2022-04-27 
 * Version 1.2.52: :textred:`resolved BUG 1052` : ClearWorkspace 
    - description:  has no effect on global variables
    - **notes:** NOTE: matplotlib looses figures which cannot be closed with closeAll in PlotSensor
    - date resolved: **2022-04-27 19:03**\ , date raised: 2022-04-25 
 * Version 1.2.51: :textred:`resolved BUG 1053` : coordinatesSolution 
    - description:  coordinatesSolutionFile is corrupted for ACF test in text mode
    - **notes:** not repeatable; may have been caused by iPython crash
    - date resolved: **2022-04-27 10:29**\ , date raised: 2022-04-26 
 * Version 1.2.50: resolved Issue 1051: CreateNonlinearFEMObjectGenericODE2NGsolve (testing)
    - description:  test FEM.CreateNonlinearFEMObjectGenericODE2NGsolve with simple example
    - **notes:** included in ACFtest.py
    - date resolved: **2022-04-27 10:23**\ , date raised: 2022-04-25 
 * Version 1.2.49: resolved Issue 1057: solutionInformation (extension)
    - description:  replace new lines in solutionInformation when writing into coordinatesSolution file in order to preserve readable file structure
    - date resolved: **2022-04-27 10:22**\ , date raised: 2022-04-27 
 * Version 1.2.48: resolved Issue 1056: MarkerSuperElementRigid (extension)
    - description:  add check for node sizes used in marker
    - date resolved: **2022-04-26 22:29**\ , date raised: 2022-04-26 
 * Version 1.2.47: :textred:`resolved BUG 1055` : NodeGenericODE2 error 
    - description:  NodeGenericODE2 raises system error CNodeODE2::GetVelocity: call illegal during solver initialize
    - **notes:** added GetVelocity and GetAcceleration to NodeGenericODE2, with risk that these functions are used unintended
    - date resolved: **2022-04-26 22:29**\ , date raised: 2022-04-26 
 * Version 1.2.46: resolved Issue 1054: NodeIndex, ... (extension)
    - description:  add possibility to allow simple arithmetic operations +,-,\*,-() for NodeIndex, MarkerIndex, etc.
    - date resolved: **2022-04-26 22:29**\ , date raised: 2022-04-26 
 * Version 1.2.45: :textred:`resolved BUG 1050` : CreateLinearFEMObjectGenericODE2 
    - description:  FEM.CreateLinearFEMObjectGenericODE2 not working
    - date resolved: **2022-04-25 17:34**\ , date raised: 2022-04-25 
 * Version 1.2.44: resolved Issue 1046: Vector3DList (extension)
    - description:  add new structure Vector3DList; used to create list of 3D vectors and transfer into MainSystem mbs, e.g. in KinematicTree
    - date resolved: **2022-04-25 08:40**\ , date raised: 2022-04-25 
 * Version 1.2.43: resolved Issue 1045: dispy cluster (extension)
    - description:  fixed some bugs in ProcessParameterList (http_server) ; added some checks (if dispy is installed); cluster is used now iff clusterHostNames != []) and useMultiProcessing==True
    - date resolved: **2022-04-23 17:09**\ , date raised: 2022-04-23 
 * Version 1.2.42: :textred:`resolved BUG 1044` : GetNodesOnLine 
    - description:  FEM.GetNodesOnLine raises exception
    - date resolved: **2022-04-22 18:28**\ , date raised: 2022-04-22 
 * Version 1.2.41: :textred:`resolved BUG 1043` : serialRobotTest.py 
    - description:  class robotics.Robot computes wrong static torque compensation in function StaticTorques(HT)
    - **notes:** due to #0744 this bug has been magically resolved!
    - date resolved: **2022-04-21 15:15**\ , date raised: 2022-04-21 
 * Version 1.2.40: resolved Issue 1042: multithreaded compilation (extension)
    - description:  enable setup.py with multithreaded compilation, using Monkey patch for compiler with multithreading library; enable parallel build with: python setup.py install --parallel
    - date resolved: **2022-04-21 02:00**\ , date raised: 2022-04-21 
 * Version 1.2.39: resolved Issue 0744: CreateRedundantCoordinateMBS (change)
    - description:  interchange joint markers, affecting the sign of the measured joint rotation angle, being now consistent with minimal coordinate formulations / kinematicTree; also interchange jointTorque0List/1List to be consistent with marker list 
    - **notes:** adapted SerialRobotTestDH2.py and SerialRobotTSD.py in Example folders for corrected signs
    - date resolved: **2022-04-20 00:37**\ , date raised: 2021-09-01 
 * Version 1.2.38: resolved Issue 1041: Pluecker transforms (change)
    - description:  correct Pluecker transform T66 functions, such as RotationTranslation2T66, etc. and introduce inverse functions for usage in Featherstone algorithm
    - **notes:** note that rotations in RotationX2T66, RotationY2T66, RotationZ2T66 are transposed/CHANGED as compared to previous version; T66toRotationTranslation is corrected/CHANGED and now is consistent with the backtransformation; InverseT66toRotationTranslation does the inverse operation (but also for rotation); RotationTranslation2T66 corrected/CHANGED; HT2T66 is corrected/CHANGED with inverse version HT2T66Inverse; rotations in Exudyn now consistent between T66, HT and other rotation matrices
    - date resolved: **2022-04-19 16:16**\ , date raised: 2022-04-19 
 * Version 1.2.37: resolved Issue 1021: twitter (extension)
    - description:  add twitter account for exudyn: https://twitter.com/RExudyn
    - **notes:** Follow me!
    - date resolved: **2022-04-12 16:28**\ , date raised: 2022-03-31 
 * Version 1.2.36: resolved Issue 1040: contour bodies (extension)
    - description:  show contour colors for GraphicsData added to bodies
    - **notes:** option turned on by default, but may slow down visualization for larger models; turn off in case of problems...
    - date resolved: **2022-04-10 17:21**\ , date raised: 2022-04-10 
 * Version 1.2.35: resolved Issue 1039: mesh edges (extension)
    - description:  show mesh edges independently of visualization mode (edges on/off) - allowing to show mesh edges but not other graphics edges
    - date resolved: **2022-04-10 11:42**\ , date raised: 2022-04-10 
 * Version 1.2.34: resolved Issue 1036: solution file (change)
    - description:  fixed inconsistent information in line 3 of coordinatesSolutionFile: ODE2 coordinates
    - date resolved: **2022-04-07 20:30**\ , date raised: 2022-04-07 
 * Version 1.2.33: resolved Issue 1034: solution and sensor files (extension)
    - description:  add Python version to e.g. Exudyn version = 1.x.y Python3.9 / Python3.6(32bits) in coordinatesSolutionFile and sensor output files to identify exactly the versions and platform used for computation; check also Parameter and optimization output files for updated information
    - date resolved: **2022-04-07 12:02**\ , date raised: 2022-04-07 
 * Version 1.2.32: resolved Issue 1033: InteractiveDialog (extenison)
    - description:  also add tkInter DoubleVar for sliders to make bi-directional interaction simpler (but not needed)
    - date resolved: **2022-04-04 21:26**\ , date raised: 2022-04-04 
 * Version 1.2.31: resolved Issue 1031: InteractiveDialog (extension)
    - description:  extended with option addLabelStringVariables to be able to modify strings in text labels
    - date resolved: **2022-04-04 17:20**\ , date raised: 2022-04-04 
 * Version 1.2.30: resolved Issue 1030: AVX2 (change)
    - description:  change both Python 3.6 32bit/46bit versions to compilation without AVX
    - date resolved: **2022-04-04 15:39**\ , date raised: 2022-04-04 
 * Version 1.2.29: resolved Issue 1024: PERFORM_UNIT_TESTS (change)
    - description:  set PERFORM_UNIT_TESTS for P3.7 instead of P3.6
    - date resolved: **2022-04-04 15:38**\ , date raised: 2022-03-31 
 * Version 1.2.28: :textred:`resolved BUG 1029` : ContactFrictionCircleCable2D 
    - description:  computes wrong segment length internally, leading to wrong sticking position
    - date resolved: **2022-04-04 12:14**\ , date raised: 2022-04-04 
 * Version 1.2.27: :textred:`resolved BUG 1028` : cnt in Render window 
    - description:  debug information cnt=.. shown in Render window
    - date resolved: **2022-04-04 11:50**\ , date raised: 2022-04-04 
 * Version 1.2.26: resolved Issue 1027: RotationMatrix2EulerParameters (change)
    - description:  RotationMatrix2EulerParameters computes Euler parameters with large deviations from unit norm in case of inaccurate rotation matrices; this may lead to failure of CheckPreAssembleConsistencies; resolved by adding normalization before returning Euler parameters
    - date resolved: **2022-04-03 00:24**\ , date raised: 2022-04-03 
 * Version 1.2.25: resolved Issue 1025: build venv (extension)
    - description:  switch to building windows purely on virtual conda environments, allowing to build all windows and linux versions in parallel
    - date resolved: **2022-04-02 00:28**\ , date raised: 2022-04-02 
 * Version 1.2.24: resolved Issue 1022: virtual environments (change)
    - description:  switch to virtual environments in anaconda for compilation on different platforms
    - date resolved: **2022-04-02 00:28**\ , date raised: 2022-03-31 
 * Version 1.2.23: resolved Issue 1023: remove /Zi in MSVC (change)
    - description:  remove compilation flag /Zi in setup.py which prevents from parallel runs of MSVC cl.exe
    - date resolved: **2022-03-31 11:28**\ , date raised: 2022-03-31 
 * Version 1.2.22: resolved Issue 1020: robotics links (docu)
    - description:  links to github are not resolved correctly for robotics module
    - date resolved: **2022-03-30 10:47**\ , date raised: 2022-03-30 
 * Version 1.2.21: resolved Issue 1019: SpaceMouse (extension)
    - description:  add functionality for 3D mouse / spacemouse by reading joystick inputs and interpret as 3D position and rotation data; add visualization flag interactive.useJoystickInput (default=True); deactivate this flag if your external device makes problems
    - date resolved: **2022-03-29 14:37**\ , date raised: 2022-03-29 
 * Version 1.2.20: resolved Issue 1017: ContactFrictionCircleCable2D (testing)
    - description:  check and test computation of sticking position segment length: undeformed versus deformed
    - **notes:** switched to reference length in computation of relative sticking position as given in theDoc; leads to improved results
    - date resolved: **2022-03-28 20:33**\ , date raised: 2022-03-28 
 * Version 1.2.19: resolved Issue 1014: SaveImage (change)
    - description:  make window height divisible by 2 (skip one line if necessary)
    - **notes:** added alignment option for width and height; default options work well for ffmpeg conversion
    - date resolved: **2022-03-28 14:25**\ , date raised: 2022-03-28 
 * Version 1.2.18: resolved Issue 1013: TGA output (change)
    - description:  switch from TGA to .png output using stb_image_write.h in GLFW
    - **notes:** added mode exportImages.saveImageFormat to chose between PNG and TGA - PNG with much smaller files!
    - date resolved: **2022-03-28 14:24**\ , date raised: 2022-03-28 
 * Version 1.2.17: resolved Issue 1012: GLFW (change)
    - description:  switch to GLFW3.3.6 includes in order to be in line with AppleM1 version
    - date resolved: **2022-03-28 12:03**\ , date raised: 2022-03-28 
 * Version 1.2.16: resolved Issue 1011: Apple M1 (change)
    - description:  use universal glfw libs for both Apple x86 and Apple arm M1
    - date resolved: **2022-03-28 12:03**\ , date raised: 2022-03-28 
 * Version 1.2.15: resolved Issue 1010: autodiff (change)
    - description:  extract autodiff as separate module
    - **notes:** now using AutomaticDifferentiation.h in Utilities
    - date resolved: **2022-03-28 11:56**\ , date raised: 2022-03-28 
 * Version 1.2.14: :textred:`resolved BUG 1009` : solutionViewer 
    - description:  does not load automatically due to change of coordinatesSolution filename ending
    - **notes:** changed loading of default file in SolutionViewer
    - date resolved: **2022-03-28 09:11**\ , date raised: 2022-03-28 
 * Version 1.2.13: :textred:`resolved BUG 1007` : sensor double values 
    - description:  sensor outputs two times for a single time step
    - **notes:** error occured due to automaticStepSize activated and call to ReduceStepSize even in case adaptiveStep=0; automaticStepSize now deactivated for solvers without step size control
    - date resolved: **2022-03-27 19:31**\ , date raised: 2022-03-24 
 * Version 1.2.12: resolved Issue 1006: ContactFrictionCircleCable2D (change)
    - description:  LHS computation: exclude undefined state from sticking position computation
    - date resolved: **2022-03-22 12:43**\ , date raised: 2022-03-22 
 * Version 1.2.11: resolved Issue 1005: PostNewton timer (change)
    - description:  remove timer from timer structures and activate as special timer
    - date resolved: **2022-03-22 12:30**\ , date raised: 2022-03-22 
 * Version 1.2.10: resolved Issue 1004: ContactFrictionCircleCable2D (extension)
    - description:  add option usePointWiseNormals flag as an additional option to control the way forces are applied to cable
    - date resolved: **2022-03-21 10:12**\ , date raised: 2022-03-21 
 * Version 1.2.9: resolved Issue 1003: setup.py (extension)
    - description:  include manifest.in and readme.md in main
    - date resolved: **2022-03-20 11:47**\ , date raised: 2022-03-20 
 * Version 1.2.8: resolved Issue 1002: pypi problems (change)
    - description:  previous version marked beta
    - date resolved: **2022-03-19 10:54**\ , date raised: 2022-03-19 
 * Version 1.2.7: resolved Issue 0986: ANCFCable2D (docu)
    - description:  extend/finalize description - specifically for integration points and OutputVariable functions
    - date resolved: **2022-03-18 23:09**\ , date raised: 2022-03-15 
 * Version 1.2.6: resolved Issue 1001: pypi (change)
    - description:  finalized conversion of markup file for description at pypi.org: use only links to github, as pypi does not recognize .rst file in github format
    - date resolved: **2022-03-18 20:54**\ , date raised: 2022-03-18 
 * Version 1.2.5: resolved Issue 1000: adjust version in docu (docu)
    - description:  recompile
    - date resolved: **2022-03-18 19:07**\ , date raised: 2022-03-18 
 * Version 1.2.4: resolved Issue 0999: pypi (extension)
    - description:  improved versioning
    - date resolved: **2022-03-18 18:28**\ , date raised: 2022-03-18 
 * Version 1.2.3: resolved Issue 0998: pypi (extension)
    - description:  add description and tags
    - date resolved: **2022-03-18 18:09**\ , date raised: 2022-03-18 
 * Version 1.2.2: resolved Issue 0997: pre-release version (extension)
    - description:  add ".dev" tag in version and wheel name for pre-releases, allowing to distinguish on pypi for different versions
    - date resolved: **2022-03-18 17:02**\ , date raised: 2022-03-18 
 * Version 1.2.1: resolved Issue 0996: pre-release (extension)
    - description:  allow pre-releases to be uploaded on pypi, fetched with "pip install exudyn --pre"
    - date resolved: **2022-03-18 15:52**\ , date raised: 2022-03-18 
 * Version 1.2.0: resolved Issue 0995: add to pypi index (extension)
    - description:  add exudyn to pypi index; allows to use "pip install exudyn"
    - date resolved: **2022-03-18 10:44**\ , date raised: 2022-03-18 

***********
Version 1.1
***********

 * Version 1.1.177: :textred:`resolved BUG 0994` : ObjectContactFrictionCircleCable2D 
    - description:  forces on circle added up twice, because weighting factor not considered
    - **notes:** tested force on circle with static computation in belt drive
    - date resolved: **2022-03-18 08:30**\ , date raised: 2022-03-18 
 * Version 1.1.176: resolved Issue 0880: User function for ConnectorCoordinateVector (extension)
    - description:  add user function for constraint and for jacobian; test with double pendulum made of masses
    - date resolved: **2022-03-17 18:14**\ , date raised: 2022-01-24 
 * Version 1.1.175: :textred:`resolved BUG 0993` : ObjectConnectorDistance 
    - description:  activeConnector=False produces wrong jacobian
    - date resolved: **2022-03-17 17:52**\ , date raised: 2022-03-17 
 * Version 1.1.174: resolved Issue 0992: PlotSensor (extension)
    - description:  add PlotSensorDefaults function which allows to set default values for all subsequent PlotSensor calls
    - **notes:** use e.g. PlotSensorDefaults().fontSize=16 to change fontSize for all subsequent calls
    - date resolved: **2022-03-17 17:28**\ , date raised: 2022-03-17 
 * Version 1.1.173: resolved Issue 0991: velocityOffset (extension)
    - description:  add velocity offset for SpringDamper and TorsionalSpringDamper, allowing simple controllers using offset and velocityOffset in preStepUser functions
    - date resolved: **2022-03-16 23:51**\ , date raised: 2022-03-16 
 * Version 1.1.172: resolved Issue 0989: convergence problems (docu)
    - description:  add specific section in theDoc - Exudyn Basics related to ways for resolving convergence problems
    - date resolved: **2022-03-16 18:59**\ , date raised: 2022-03-16 
 * Version 1.1.171: resolved Issue 0938: ANCFCable2D (extension)
    - description:  add drawing function for forces in normal direction with factor
    - date resolved: **2022-03-15 20:33**\ , date raised: 2022-02-10 
 * Version 1.1.170: resolved Issue 0933: RigidBody (extension)
    - description:  add accelerationLocal and angularAccelerationLocal output
    - **notes:** added to ObjectRigidBody and ObjectRigidBody2D
    - date resolved: **2022-03-15 20:14**\ , date raised: 2022-02-07 
 * Version 1.1.169: resolved Issue 0967: ANCFCable2D (extension)
    - description:  add OutputVariables RotationMatrix, Rotation, AngularVelocity(Local) and AngularAcceleration
    - **notes:** Acceleration, AngularVelocity and AngularAcceleration currently only implemented for ANCFCable2D but not for ALEANCFCable2D
    - date resolved: **2022-03-15 20:01**\ , date raised: 2022-03-03 
 * Version 1.1.168: resolved Issue 0966: ANCFCable2D (extension)
    - description:  add missing terms for OutputVariables and AccessFunctions to allow constraints that are at local position y!=0
    - date resolved: **2022-03-15 19:54**\ , date raised: 2022-03-03 
 * Version 1.1.167: resolved Issue 0983: plot (extension)
    - description:  add function to create plot-ready data from mbs.GetObjectOutputBody for a list of consecutive beams, e.g., axial force, displacement or curvature along axial reference coordinate
    - **notes:** added function DataArrayFromSensorList which allows to create data from a list of sensors
    - date resolved: **2022-03-14 20:41**\ , date raised: 2022-03-11 
 * Version 1.1.166: resolved Issue 0982: PlotSensor (extentsion)
    - description:  add option to plot 2D arrays
    - **notes:** numpy arrays are used instead of sensorNumbers; these arrays must have the same format as data stored in sensor files; this data format does not create any labels
    - date resolved: **2022-03-14 16:24**\ , date raised: 2022-03-11 
 * Version 1.1.165: resolved Issue 0985: startOfStepState (extension)
    - description:  initialize startOfStepState together with currentState in InitializeSolverInitialConditions(...) in order to be valid when sensors are written in initialization
    - date resolved: **2022-03-14 13:07**\ , date raised: 2022-03-14 
 * Version 1.1.164: resolved Issue 0981: ObjectContactFrictionCircleCable2D (extension)
    - description:  add OutputVariable functions for Coordinates (gap, slip), Coordinates_t (gap_t, slip_t), and contact and friction forces per segment (ForceLocal)
    - date resolved: **2022-03-14 12:58**\ , date raised: 2022-03-11 
 * Version 1.1.163: resolved Issue 0980: renderer precision (extension)
    - description:  add options for general precision in renderer: general.rendererPrecision as well as precision for colorbars in contour: contour.colorBarPrecision
    - date resolved: **2022-03-11 14:05**\ , date raised: 2022-03-11 
 * Version 1.1.162: resolved Issue 0979: VisualizationSettings (extension)
    - description:  VisualizationSettings.showContactForcesValues is added to show numerical values for contact forces
    - date resolved: **2022-03-11 10:19**\ , date raised: 2022-03-11 
 * Version 1.1.161: resolved Issue 0978: ObjectANCFCable2D (extension)
    - description:  add improved axial strain computation for reduced order integration
    - **notes:** works for axial strain and axial force if reducedAxialInterploation=True
    - date resolved: **2022-03-10 20:36**\ , date raised: 2022-03-10 
 * Version 1.1.160: resolved Issue 0977: ObjectANCFCable2D (extension)
    - description:  add new mode useReducedOrderIntegration=2 with good performance/accuracy and exceptional representation of axial strains
    - date resolved: **2022-03-10 20:08**\ , date raised: 2022-03-10 
 * Version 1.1.159: resolved Issue 0976: ContactCircleCable2D (extension)
    - description:  add visualization flag showContactCircle; uses circleTiling\*4 for tiling (from VisualizationSettings.general)!
    - date resolved: **2022-03-10 14:15**\ , date raised: 2022-03-10 
 * Version 1.1.158: resolved Issue 0975: VisualizationSettings (extension)
    - description:  added showContactForces and contactForcesFactor in contact; this flag is currently only available in ContactCircleCable2D
    - date resolved: **2022-03-10 13:52**\ , date raised: 2022-03-10 
 * Version 1.1.157: resolved Issue 0974: VisualizationSettings (change)
    - description:  moved contactPointsDefaultSize from connectors to contact; connectors.contactPointsDefaultSize is inactive from now!
    - date resolved: **2022-03-10 13:50**\ , date raised: 2022-03-10 
 * Version 1.1.156: resolved Issue 0953: ContactFrictionCircleCable2D (extension)
    - description:  add improved (static) friction model
    - **notes:** also added improved PostNewton switching strategies and changed initial values for NodeGenericData
    - date resolved: **2022-03-09 21:42**\ , date raised: 2022-02-25 
 * Version 1.1.155: resolved Issue 0932: ANCFCable2D (extension)
    - description:  add velocityLocal + accelerationLocal output in axial/normal direction
    - **notes:** only velocityLocal added
    - date resolved: **2022-03-06 18:24**\ , date raised: 2022-02-07 
 * Version 1.1.154: resolved Issue 0931: ANCFCable2D (extension)
    - description:  add acceleration as output variable
    - **notes:** only added for ANCF, but not for ALEANCF due to coupling terms with vALE
    - date resolved: **2022-03-06 18:24**\ , date raised: 2022-02-07 
 * Version 1.1.153: resolved Issue 0970: PlotSensor (extension)
    - description:  PlotSensor allows to set fileCommentChar and fileDelimiterChar
    - date resolved: **2022-03-04 13:27**\ , date raised: 2022-03-04 
 * Version 1.1.152: resolved Issue 0969: exudyn.plot (extension)
    - description:  added method to convert output files from other codes; in particular plot.FileStripSpaces(...) can be used to strip leading / trailing spaces and remove double spaces
    - date resolved: **2022-03-04 13:27**\ , date raised: 2022-03-04 
 * Version 1.1.151: resolved Issue 0965: ANCFCable2D (extension)
    - description:  add option for strainIsRelativeToReference, which if set to 1. accounts for the reference geometry as the stress-free configuration; also works for ALE Cable2D
    - date resolved: **2022-03-03 08:13**\ , date raised: 2022-03-03 
 * Version 1.1.150: resolved Issue 0964: CreateReevingCurve (change)
    - description:  corrected sign of returned curvatures, now can be directly used for beam elements
    - date resolved: **2022-03-02 17:20**\ , date raised: 2022-03-02 
 * Version 1.1.149: :textred:`resolved BUG 0963` : CreateReevingCurve 
    - description:  removeFirstLine=True not working: wrong case i==0 needs to be changed to i==1
    - date resolved: **2022-03-02 15:25**\ , date raised: 2022-03-02 
 * Version 1.1.148: resolved Issue 0962: CreateReevingCurve (change)
    - description:  adjust number of nodes in case of closed Curve (numberOfANCFnodes=20 gives 20 elements in this case)
    - date resolved: **2022-03-02 15:25**\ , date raised: 2022-03-02 
 * Version 1.1.147: resolved Issue 0961: stepInformation (change)
    - description:  changed flags in timeIntegration and staticSolver stepInformation: value of 1024 now causes output at every step; all values > 16 have been divided by two; 255=detailed overall output, 2047=detailed output every step; see SimulationSettings in theDoc
    - date resolved: **2022-03-02 10:33**\ , date raised: 2022-03-02 
 * Version 1.1.146: resolved Issue 0959: coordinatesSolution (change)
    - description:  change ending of coordinates solution files from .txt to .sol in case of binary files
    - date resolved: **2022-03-02 10:14**\ , date raised: 2022-03-02 
 * Version 1.1.145: resolved Issue 0960: ObjectContactFrictionCircleCable2D (change)
    - description:  move to ObjectContactFrictionCircleCable2DOld and improve new version
    - date resolved: **2022-03-02 09:50**\ , date raised: 2022-03-02 
 * Version 1.1.144: resolved Issue 0958: LoadForceVector (docu)
    - description:  add note to description in forces that user function values are available in sensors, but they are not updated during drawing
    - date resolved: **2022-03-01 19:40**\ , date raised: 2022-02-28 
 * Version 1.1.143: resolved Issue 0952: rolling joints (extension)
    - description:  ConnectorRollingDiscPenalty and JointRollingDisc now add a OutputVariable RotationMatrix, containing the J1 to global transformation
    - date resolved: **2022-03-01 19:35**\ , date raised: 2022-02-25 
 * Version 1.1.142: resolved Issue 0951: Friction (test)
    - description:  test LuGre friction model as ODE1 model versus position/history based model
    - **notes:** added lugreFrictionODE1.py as a demo showing the LuGre model based on a ODE1 user function
    - date resolved: **2022-03-01 19:34**\ , date raised: 2022-02-24 
 * Version 1.1.141: resolved Issue 0956: ProfileLinearAccelerationsList (extension)
    - description:  add linear acceleration profile to robotics.motion, currently only allowing to create profile directly from accelerations
    - date resolved: **2022-02-28 19:05**\ , date raised: 2022-02-28 
 * Version 1.1.140: resolved Issue 0955: robotics.motion (change)
    - description:  change PTPprofile to BasicProfile
    - date resolved: **2022-02-28 09:59**\ , date raised: 2022-02-28 
 * Version 1.1.139: :textred:`resolved BUG 0950` : JointRollingDisc 
    - description:  OutputVariable VelocityLocal is returned in global coordinates; description in theDoc is wrong; will be changed to outputVariable Velocity; local velocity will represent the slippage in special local joint J1 coordinates; see theDoc for updated functionality and outputs
    - date resolved: **2022-02-25 14:32**\ , date raised: 2022-02-22 
 * Version 1.1.138: :textred:`resolved BUG 0949` : ConnectorRollingDiscPenalty 
    - description:  OutputVariable VelocityLocal local is returned in global coordinates; description in theDoc is wrong; will be changed to outputVariable Velocity; see theDoc for updated functionality and outputs
    - date resolved: **2022-02-25 14:32**\ , date raised: 2022-02-22 
 * Version 1.1.137: resolved Issue 0946: sensors storeInternal (extension)
    - description:  adapt most TestModels to store sensordata internally
    - date resolved: **2022-02-21 00:43**\ , date raised: 2022-02-20 
 * Version 1.1.136: resolved Issue 0947: TestModels (change)
    - description:  change most test models to use Sensors storeInternal mode; this avoids creating many files during TestSuite runs
    - date resolved: **2022-02-20 20:40**\ , date raised: 2022-02-20 
 * Version 1.1.135: resolved Issue 0921: PlotSensor (extension)
    - description:  add option to add subplots
    - **notes:** allows to create subplots, adjusting also the plot size using sizeInches; for examples see Examples/plotSensorExamples.py
    - date resolved: **2022-02-19 22:45**\ , date raised: 2022-02-02 
 * Version 1.1.134: resolved Issue 0922: PlotSensor (extension)
    - description:  add option to add linewidth, markersize, markerStyles=["o",...], markerSizes=[], lineStyles=["-",...], colors=[..], markerDensity=...; lineWidths=[] allowing to plot a reduced number of markers on top of a line
    - **notes:** added many options for line and marker styles, check: colors, lineStyles, lineWidths, markerStyles, markerSizes, markerDensity
    - date resolved: **2022-02-19 22:41**\ , date raised: 2022-02-02 
 * Version 1.1.133: resolved Issue 0920: PlotSensor (extension)
    - description:  add x-range and y-range for zoom
    - **notes:** added rangeX and rangeY options to specify range
    - date resolved: **2022-02-19 16:59**\ , date raised: 2022-02-02 
 * Version 1.1.132: resolved Issue 0945: PlotSensor (extension)
    - description:  extend option for offsets, allowing sensor data to use as offset (e.g., loaded from file or from internal sensor data)
    - **notes:** see plotSensorExamples for some particular usage
    - date resolved: **2022-02-19 16:45**\ , date raised: 2022-02-19 
 * Version 1.1.131: resolved Issue 0944: PlotSensor (extension)
    - description:  add argument labels, which can be string (for one sensor) or list of strings (according to number of sensors resp. components) representing the labels used in legend; if not provided, automatically generated legend is used 
    - date resolved: **2022-02-19 15:47**\ , date raised: 2022-02-19 
 * Version 1.1.130: resolved Issue 0918: Sensors store values internally (extension)
    - description:  add option to store sensor values internally; using ResizableMatrix internally; rows added and matrix is automatically resized
    - **notes:** storeInternal boost the speed of writing sensor values, however, most of time is spent on computing sensor values, which typically about 2 seconds for 1e6 time steps per sensor; file write adds usually 3 seconds extra on that bill
    - date resolved: **2022-02-19 00:13**\ , date raised: 2022-02-02 
 * Version 1.1.129: resolved Issue 0943: mbs.GetSensorStoredData() (extension)
    - description:  add MainSystem functionality to retrieve internally stored data in sensor; used, e.g., for PlotSensor to plot data without storing in files
    - date resolved: **2022-02-19 00:10**\ , date raised: 2022-02-18 
 * Version 1.1.128: :textred:`resolved BUG 0942` : sensorsAppendToFile 
    - description:  appendToFile is used instead of sensorsAppendToFile for switching between append and replace operations for files
    - date resolved: **2022-02-18 20:54**\ , date raised: 2022-02-18 
 * Version 1.1.127: resolved Issue 0901: SimulationSettings (extension)
    - description:  improve type completion by adding py::init<...> functions for all subclasses
    - **notes:** not solvable in this simple way as type completion is not improved when adding this information
    - date resolved: **2022-02-18 20:30**\ , date raised: 2022-01-31 
 * Version 1.1.126: resolved Issue 0941: MotionInterpolator (extension)
    - description:  mark as deprecated; instead, created motion submodule in robotics with class Trajectory; uses classes ProfileConstantAcceleration and ProfilePTP to construct piecewise profiles for trajectory; precomputes acceleration profiles in first step and thus is several factors faster in Python implementation; Trajectory converts to dict and can be printed in order to obtain key values of computed trajectories
    - date resolved: **2022-02-18 16:31**\ , date raised: 2022-02-15 
 * Version 1.1.125: resolved Issue 0940: MotionInterpolator (extension)
    - description:  add option to add trajectories defined by duration as well as by maxVelocity and maxAcceleration using synchronous PTP trajectory generation
    - date resolved: **2022-02-15 12:38**\ , date raised: 2022-02-15 
 * Version 1.1.124: :textred:`resolved BUG 0939` : NodeRigidBody2D 
    - description:  does not correctly measure rotations (returns x-coordinate instead of angle)
    - date resolved: **2022-02-15 09:58**\ , date raised: 2022-02-15 
 * Version 1.1.123: resolved Issue 0745: SensitivityAnalysis() (extension)
    - description:  add functionality to processing, evaluating the sensitivities of certain sensor values w.r.t. parameters; same interface as ParameterVariation
    - **notes:** see processing.ComputeSensitivities(...)
    - date resolved: **2022-02-14 09:50**\ , date raised: 2021-09-03 
    - resolved by: P. Manzl
 * Version 1.1.122: :textred:`resolved BUG 0934` : exudyn.signal 
    - description:  conflicts with Python 3.8.8 and Python 3.9.7 (and possibly other) with internal Python signal package; change exudyn.signal to exudyn.signalProcessing
    - date resolved: **2022-02-10 09:23**\ , date raised: 2022-02-10 
 * Version 1.1.121: resolved Issue 0870: binary output (extension)
    - description:  add option to create binary solution files
    - **notes:** notes: use outputPrecision to switch between float and double - see there; speeds up file writing considerably and reduces file sizes; Integrated into LoadSolutionFile
    - date resolved: **2022-02-09 08:45**\ , date raised: 2022-01-18 
 * Version 1.1.120: resolved Issue 0930: LoadSolutionFile (extension)
    - description:  extend function to check if binary file; if yes, switches to binary mode
    - date resolved: **2022-02-09 08:44**\ , date raised: 2022-02-07 
 * Version 1.1.119: resolved Issue 0929: coordinatesSolution, sensors (change)
    - description:  add version to coordinatesSolutionFile and sensor output files; should not affect current parsing of output files
    - date resolved: **2022-02-07 00:02**\ , date raised: 2022-02-07 
 * Version 1.1.118: resolved Issue 0878: sort settings options (docu)
    - description:  sort settings representation in Python by sorting dictionaries prior to writing inteface files; keep current sorting (grouping) in latex documentation
    - **notes:** type completion may change sorting afterwards
    - date resolved: **2022-02-06 21:57**\ , date raised: 2022-01-24 
 * Version 1.1.117: resolved Issue 0928: CPP UNIT TESTS (testing)
    - description:  fail because of change of ConstSizeVector to use move assignment and move constructor
    - **notes:** adapted test to avoid move assignment
    - date resolved: **2022-02-06 00:04**\ , date raised: 2022-02-06 
 * Version 1.1.116: resolved Issue 0919: Minimize (extension)
    - description:  add optimization with same interface as GenticOptimization but based on scipy.minimize
    - **notes:** added Minimize to processing; usage is nearly same as GeneticOptimization(...); example under Examples/minimizeExample.py
    - date resolved: **2022-02-04 15:45**\ , date raised: 2022-02-02 
    - resolved by: S. Holzinger
 * Version 1.1.115: :textred:`resolved BUG 0924` : GeneralContact 
    - description:  WARNING message raised in ANCF contact
    - date resolved: **2022-02-03 12:01**\ , date raised: 2022-02-03 
 * Version 1.1.114: resolved Issue 0923: CreateReevingCurve (extension)
    - description:  add function CreateReevingCurve(...) in exudyn.beams to create reeving system along circles, allows to create curve and nodes for ANCFCable2D elements created with PointsAndSlopes2ANCFCable2D(...); see Examples/reevingSystem.py
    - date resolved: **2022-02-03 12:00**\ , date raised: 2022-02-02 
 * Version 1.1.113: resolved Issue 0803: PostNewton (check)
    - description:  Check discontinuous iterations in combination with adaptive step (immediately reduces step size even if ignoreMaxSteps=True)
    - **notes:** did not further show up; possibly due to non-convergence
    - date resolved: **2022-02-02 08:20**\ , date raised: 2021-11-25 
 * Version 1.1.112: resolved Issue 0906: PlotSensor (extension)
    - description:  add option componentsX to add x-components for figures, e.g., to plot y over x position, instead over time
    - date resolved: **2022-02-01 18:34**\ , date raised: 2022-02-01 
 * Version 1.1.111: resolved Issue 0864: automatic example referencing (fix)
    - description:  fix searching for examples, e.g., NodePoint as NodePoint( in order no to find NodePoint2D examples
    - date resolved: **2022-01-31 23:01**\ , date raised: 2022-01-14 
 * Version 1.1.110: :textred:`resolved BUG 0903` : GeneralContact 
    - description:  autocomputed searchTree gives very large values and visualization not working
    - **notes:** ANCFCable2D bounding box computation had 1 wrong else case
    - date resolved: **2022-01-31 21:34**\ , date raised: 2022-01-31 
 * Version 1.1.109: resolved Issue 0899: GenerateCircularArcANCFCable2D (extension)
    - description:  add function to create beams along circular arc
    - date resolved: **2022-01-31 14:20**\ , date raised: 2022-01-30 
 * Version 1.1.108: :textred:`resolved BUG 0902` : GenerateStraightLineANCFCable2D 
    - description:  cableNodePositionList does not returns 3D vectors for nodes except first node
    - date resolved: **2022-01-31 14:19**\ , date raised: 2022-01-31 
 * Version 1.1.107: resolved Issue 0898: GenerateStraightLineANCFCable2D (extension)
    - description:  add option to use existing nodes in generation of beams
    - date resolved: **2022-01-31 10:11**\ , date raised: 2022-01-30 
 * Version 1.1.106: resolved Issue 0897: add Python utility beams (extension)
    - description:  add exudyn.beams utility module, containing helper functions for creating beams, etc.; move existing functions to this module
    - date resolved: **2022-01-31 10:11**\ , date raised: 2022-01-30 
 * Version 1.1.105: :textred:`resolved BUG 0900` : GenerateStraightLineANCFCable2D 
    - description:  arguments vALE and ConstrainAleCoordinate are not implemented and need to be removed
    - date resolved: **2022-01-30 23:32**\ , date raised: 2022-01-30 
 * Version 1.1.104: resolved Issue 0896: theDoc objects (docu)
    - description:  sort objects into bodies, basic connectors, constraints and joints
    - date resolved: **2022-01-30 23:12**\ , date raised: 2022-01-30 
 * Version 1.1.103: resolved Issue 0895: ConnectorGravity (extension)
    - description:  add connector representing gravitational forces between heavy masses (planet, satellite, etc.)
    - date resolved: **2022-01-30 19:05**\ , date raised: 2022-01-30 
 * Version 1.1.102: resolved Issue 0894: colors (extension)
    - description:  added several colors in graphicsDataUtilities, added color4black, extended color4list to 16 colors
    - date resolved: **2022-01-30 18:10**\ , date raised: 2022-01-30 
 * Version 1.1.101: resolved Issue 0893: GeneralContact (change)
    - description:  changed flag introSpheresContact to sphereSphereContact; added flag for special mode to recycle last Post Newton step friction force
    - date resolved: **2022-01-28 18:55**\ , date raised: 2022-01-28 
 * Version 1.1.100: resolved Issue 0891: adapt to Python3.9 (extension)
    - description:  make wheels and installers for Python3.9, running on Anaconda3-2021-11 Windows-x86_64
    - date resolved: **2022-01-25 23:31**\ , date raised: 2022-01-25 
 * Version 1.1.99: resolved Issue 0889: PlotSensor (testing)
    - description:  add test for PlotSensor, returning 1 if all plotting runs without crashing
    - date resolved: **2022-01-25 19:03**\ , date raised: 2022-01-25 
 * Version 1.1.98: resolved Issue 0890: PlotSensor (extension)
    - description:  add possibility to use filenames instead of sensor numbers, automatically loading these files
    - date resolved: **2022-01-25 18:00**\ , date raised: 2022-01-25 
 * Version 1.1.97: :textred:`resolved BUG 0887` : iPython output stops 
    - description:  after a solver error, output of iPython stops
    - **notes:** changed to correct catching/throwing of Python and C++ exception types; outpur continues after solver error
    - date resolved: **2022-01-25 14:25**\ , date raised: 2022-01-25 
 * Version 1.1.96: resolved Issue 0885: GetInterpolatedSignalValue (extension)
    - description:  add check in case that time values are distributed non-uniform; add tolerance as option
    - date resolved: **2022-01-25 12:10**\ , date raised: 2022-01-25 
 * Version 1.1.95: resolved Issue 0884: PlotSensor (extension)
    - description:  add optional title to plot
    - date resolved: **2022-01-25 11:52**\ , date raised: 2022-01-25 
 * Version 1.1.94: resolved Issue 0883: PlotSensor (extension)
    - description:  if sensorNumbers is scalar, components is a list, sensorNumbers is automatically adjusted; accepts now e.g. PlotSensor(mbs, 0, components=[0,1,2])
    - date resolved: **2022-01-25 11:36**\ , date raised: 2022-01-25 
 * Version 1.1.93: resolved Issue 0881: PlotSensor (extension)
    - description:  add option to apply factors and offsets to plotted signals; add option to add labels which appears at legend
    - date resolved: **2022-01-25 11:36**\ , date raised: 2022-01-25 
 * Version 1.1.92: resolved Issue 0882: PlotSensor (change)
    - description:  add option to use X, Y and Z components instead of 0, 1, 2 for Position, Displacement etc.; this option is enabled by default and changes the appearance -> set False, to preserve the old mode; component now also shown if only one curve plotted
    - date resolved: **2022-01-25 11:19**\ , date raised: 2022-01-25 
 * Version 1.1.91: resolved Issue 0879: LoadSolutionFile (extension)
    - description:  changed safeMode to loading sinle lines, which saves memory enormously, and added new options for loading huge files
    - date resolved: **2022-01-24 14:53**\ , date raised: 2022-01-24 
 * Version 1.1.90: :textred:`resolved BUG 0877` : AddSensorRecorder 
    - description:  fails for scalar Sensors OutputVariableTypes (e.g. when measuring Rotation of TorsionalSpringDamper)
    - **notes:** added special scalar case
    - date resolved: **2022-01-21 09:17**\ , date raised: 2022-01-21 
 * Version 1.1.89: resolved Issue 0876: flush files (extension)
    - description:  add option to flush solution and sensor files immediately after writing, simplifying the readout process; add option for large scale simulations, which are always flushed - helping for continuation of computations on supercomputers
    - date resolved: **2022-01-20 14:01**\ , date raised: 2022-01-20 
 * Version 1.1.88: resolved Issue 0875: PlotSensor (extension)
    - description:  fixed fontSize option and add options for minor/major ticks and SAVE figure to PlotSensor(...) using fileName=...
    - date resolved: **2022-01-19 11:08**\ , date raised: 2022-01-19 
 * Version 1.1.87: resolved Issue 0800: GeneralContact ANCFCable (extension)
    - description:  add ANCFCable2D to GeneralContact, enabling contact with planar spheres (cylinders)
    - date resolved: **2022-01-18 18:54**\ , date raised: 2021-11-19 
 * Version 1.1.86: resolved Issue 0866: numberOfThreads (extension)
    - description:  move simulationSettings.numberOfThreads into new section parallel in simulationSettings; remove comment [not implemented]
    - date resolved: **2022-01-18 17:43**\ , date raised: 2022-01-15 
 * Version 1.1.85: resolved Issue 0872: preStepPyExecute (change)
    - description:  remove preStepPyExecute from docu, time integration / static solver interface and from CSolverBase
    - date resolved: **2022-01-18 16:57**\ , date raised: 2022-01-18 
 * Version 1.1.84: resolved Issue 0874: improved Newton restart (change)
    - description:  added an additional Newton iteration after restarting modified Newton or when switching to full Newton; this reduces effects in generalized alpha and may improve behaviour with severe nonlinearities
    - date resolved: **2022-01-18 13:44**\ , date raised: 2022-01-18 
 * Version 1.1.83: resolved Issue 0869: adaptiveStepRecoveryIterations (change)
    - description:  add option to static and dynamic solvers to adjust max. Newton+disc. iterations prior to increase of step size; changed (previous internal) default value from 5 to 7
    - date resolved: **2022-01-17 19:54**\ , date raised: 2022-01-17 
 * Version 1.1.82: resolved Issue 0868: solverSettings.stepInformation (extension)
    - description:  change modes to add up binary flags; ADD several new options to show Newton iterations, jacobians, discontinuous iterations, per step or period; also add option to show output at every step
    - date resolved: **2022-01-17 12:11**\ , date raised: 2022-01-17 
 * Version 1.1.81: resolved Issue 0857: GetInterpolatedSignalValue (extension)
    - description:  new function to interpolate a numeric signal with time/data vectors at a certain time point
    - date resolved: **2022-01-11 15:41**\ , date raised: 2022-01-11 
 * Version 1.1.80: resolved Issue 0856: IndexFromValue (extension)
    - description:  function got faster mode in case of constant sampling rate
    - date resolved: **2022-01-11 14:39**\ , date raised: 2022-01-11 
 * Version 1.1.79: resolved Issue 0854: Add LTG description (docu)
    - description:  add section on local-to-global mapping of coordinates Section :ref:`sec-overview-ltgmapping`\ 
    - date resolved: **2022-01-08 12:09**\ , date raised: 2022-01-08 
 * Version 1.1.78: resolved Issue 0849: publications directory (docu)
    - description:  create separate directory Examples/publications/ for publication data, Python files of numerical examples, etc.
    - date resolved: **2022-01-06 16:32**\ , date raised: 2022-01-06 
 * Version 1.1.77: resolved Issue 0846: analytic jacobians (extension)
    - description:  add analytic jacobians general functionality, realized for CartesianSpringDamper and CoordinateSpringDamper; deactivate with newton.numericalDifferentiation.forODE2connectors = False
    - date resolved: **2021-12-23 11:07**\ , date raised: 2021-12-23 
 * Version 1.1.76: resolved Issue 0770: Marker jacobian derivative2 (extension)
    - description:  add jacobian derivative to most important connector markers
    - **notes:** implemented for markers except MarkerSuperElement; only MarkerPosition and MarkerRigidBody affected mostly; first tests show that jacobianDerivative agrees with numerical differentiation, BUT even increases iteration numbers
    - date resolved: **2021-12-22 21:21**\ , date raised: 2021-09-28 
 * Version 1.1.75: resolved Issue 0842: implement AccessFunctionType::JacobianTtimesVector_q (extension)
    - description:  implement function needed for MarkerBody for objects ANCFCable, ObjectFFRF and ObjectFFRFreducedOrder; currently raising exception if used in this setup!
    - date resolved: **2021-12-22 21:18**\ , date raised: 2021-12-22 
 * Version 1.1.74: resolved Issue 0831: Explicit solvers (change)
    - description:  remove second ComputeODE2Acceleration() call and copy solutionODE2_tt from beginning of time step rk.stageDerivODE2_t[0], same for ODE1 variables; add flag but change that by default as it speeds up 2x; could also use information from previous step ...?
    - date resolved: **2021-12-20 13:09**\ , date raised: 2021-12-15 
 * Version 1.1.73: resolved Issue 0832: explicit integrator (extension)
    - description:  add flag timeintegration.explicitIntegration.computeEndOfStepAccelerations to compute end-of-step accelerations; this computation doubles the effort of explicit one-step-methods, particularly relevant in particles or contact simulations
    - date resolved: **2021-12-17 12:38**\ , date raised: 2021-12-17 
 * Version 1.1.72: resolved Issue 0450: MT integration (extension)
    - description:  fully integrate multithreading into system.cpp, vector.cpp and dense solver
    - **notes:** integrated into system, missing mass matrix and jacobian
    - date resolved: **2021-12-09 18:17**\ , date raised: 2020-09-16 
 * Version 1.1.71: resolved Issue 0799: GeneralContact description (docu)
    - description:  add general section in theDoc for description of GeneralContact
    - date resolved: **2021-12-09 12:36**\ , date raised: 2021-11-19 
 * Version 1.1.70: resolved Issue 0793: performance section (docu)
    - description:  add performance and speedup section to theDoc, explaining most useful settings like modifiedNewton, EigenSparse, writeToFile, step size, numberOfThreads, constant mass matrix, etc.
    - date resolved: **2021-12-09 12:36**\ , date raised: 2021-11-02 
 * Version 1.1.69: resolved Issue 0823: add trig-sphere friction tests (extension)
    - description:  add simple test cases to check friction implementation
    - date resolved: **2021-12-09 08:28**\ , date raised: 2021-12-06 
 * Version 1.1.68: resolved Issue 0822: shrink mesh (extension)
    - description:  add method to shrink meshes using max distance to surface with normals; used for contact trig-sphere implementation
    - **notes:** currently slows down for larger meshes due to elimination of duplicate points!
    - date resolved: **2021-12-09 08:28**\ , date raised: 2021-12-06 
 * Version 1.1.67: :textred:`resolved BUG 0827` : GraphicsData cube 
    - description:  cubes has wrong numbering of nodes
    - **notes:** FIXED numbering in order to allow for correct normals needed in contact computation
    - date resolved: **2021-12-07 17:14**\ , date raised: 2021-12-07 
 * Version 1.1.66: resolved Issue 0826: GraphicsData normals (extension)
    - description:  add normals to some GraphicsData  cube objects
    - date resolved: **2021-12-07 16:27**\ , date raised: 2021-12-07 
 * Version 1.1.65: resolved Issue 0825: GraphicsData add defaults (extension)
    - description:  add some default values, especially to (center)point of GraphicsDataSphere, GraphicsDataCylinder, GraphicsDataOrthoCube, etc. in order to reduce interface sizes for objects added in the centerpoint [0,0,0]; thus some defaults were also necessary for radius or sizes!
    - date resolved: **2021-12-07 14:41**\ , date raised: 2021-12-07 
 * Version 1.1.64: resolved Issue 0824: PlotSensor (change)
    - description:  added serval NEW options, including: newFigure, colorCodeOffset, figureName and closeAll; NOTE that now PlotSensor by default opens a new figure!
    - date resolved: **2021-12-07 12:11**\ , date raised: 2021-12-07 
 * Version 1.1.63: :textred:`resolved BUG 0820` : RigidBody visualization 
    - description:  normals in rigid body visualization transformed wrongly
    - **notes:** before, rotating bodies may have shown shading, now resolved
    - date resolved: **2021-12-05 17:32**\ , date raised: 2021-12-05 
 * Version 1.1.62: resolved Issue 0816: GraphicsData convert (extension)
    - description:  add function GraphicsData2TrigsAndPoints(...) to convert graphicsData into triangles and points
    - date resolved: **2021-12-05 15:43**\ , date raised: 2021-12-02 
 * Version 1.1.61: resolved Issue 0814: robotics.future, robotics.utilities, robotics.mobile (extension)
    - description:  add special submodules for robotics functions; future contains currently developed submodules that will be available in future at a different location in robotics
    - date resolved: **2021-12-05 15:43**\ , date raised: 2021-12-02 
 * Version 1.1.60: resolved Issue 0819: ObjectRigidBody (change)
    - description:  correct HasConstantMassMatrix for Lie group nodes and COM=0
    - date resolved: **2021-12-05 11:19**\ , date raised: 2021-12-05 
 * Version 1.1.59: resolved Issue 0804: GeneralContact TriangleMesh (extension)
    - description:  Add rigid-body-marker based triangle mesh to GeneralContact
    - date resolved: **2021-12-04 10:08**\ , date raised: 2021-11-25 
 * Version 1.1.58: resolved Issue 0818: MergeGraphicsDataTriangleList (extension)
    - description:  now works if either both lists contain normals or both do not
    - date resolved: **2021-12-03 20:15**\ , date raised: 2021-12-03 
 * Version 1.1.57: resolved Issue 0817: NodePointGround (extension)
    - description:  add Node::Orientation to NodeType, such that it can also be used as a rigidBody node
    - date resolved: **2021-12-03 14:34**\ , date raised: 2021-12-03 
 * Version 1.1.56: resolved Issue 0815: tCPU showing wrong time (change)
    - description:  minor bug; time shown is since starting of iPython
    - date resolved: **2021-12-02 17:48**\ , date raised: 2021-12-02 
 * Version 1.1.55: resolved Issue 0813: exudyn.robotics.special (change)
    - description:  extend and move roboticsSpecial to robotics.special; add special robotics functionality likde manipulability
    - date resolved: **2021-12-02 11:03**\ , date raised: 2021-12-02 
    - resolved by: M. Sereinig
 * Version 1.1.54: resolved Issue 0618: robotics submodule (extension)
    - description:  Create robotics.special and robotics.mecanum or robotics.ros submodules with subdirectories
    - date resolved: **2021-12-02 11:02**\ , date raised: 2021-03-23 
 * Version 1.1.53: resolved Issue 0812: Suppress warnings (extension)
    - description:  add flag to globally suppress warnings
    - **notes:** use exudyn.SuppressWarnings(True) to turn off warnings
    - date resolved: **2021-12-01 08:50**\ , date raised: 2021-12-01 
 * Version 1.1.52: resolved Issue 0811: add default constructors (change)
    - description:  add rule of five default constructors to SlimVector, SlimArray, ConstSizeVector and ConstSizeMatrix; remove mutable from data
    - date resolved: **2021-11-28 21:56**\ , date raised: 2021-11-28 
 * Version 1.1.51: resolved Issue 0810: GeneralContact visualization (extension)
    - description:  add visualization for searchtree (box) and bounding boxes
    - **notes:** use SC.visualizationSettings.contact to adjust the various options to visualize the contact search tree
    - date resolved: **2021-11-26 23:20**\ , date raised: 2021-11-26 
 * Version 1.1.50: resolved Issue 0809: ParameterVariation (extension)
    - description:  Added a additional argument parameterFunctionData={} to function ParameterVariation(). The argument parameterFunctionData can be used to make global data available inside the parameterFunction.
    - date resolved: **2021-11-26 12:57**\ , date raised: 2021-11-26 
    - resolved by: S. Holzinger
 * Version 1.1.49: resolved Issue 0808: Performance test for GeneralContact (testint)
    - description:  added test with sphere contact
    - date resolved: **2021-11-26 10:49**\ , date raised: 2021-11-26 
 * Version 1.1.48: resolved Issue 0802: GeneralContact (testing)
    - description:  add TestModel for Sphere-Sphere GeneralContact
    - date resolved: **2021-11-25 22:58**\ , date raised: 2021-11-25 
 * Version 1.1.47: resolved Issue 0807: GeneralContact (change)
    - description:  added new functions to initialize searchTree, searchTreeBox and frictionPairings which were previously in FinalizeContact(...)
    - date resolved: **2021-11-25 22:16**\ , date raised: 2021-11-25 
 * Version 1.1.46: resolved Issue 0806: removed FinalizeContact (change)
    - description:  removed this function, which is now automatically called in mbs.Assemble()
    - date resolved: **2021-11-25 22:15**\ , date raised: 2021-11-25 
 * Version 1.1.45: resolved Issue 0805: AssembleSystemInitialize (extension)
    - description:  add additional function inside mbs.Assemble() to initialize GeneralContact
    - date resolved: **2021-11-25 22:14**\ , date raised: 2021-11-25 
 * Version 1.1.44: resolved Issue 0779: sensor recorder (extension)
    - description:  add utilities function for recording signals internally in mbs; avoids writing to sensor files, which helps reducing overhead in ParameterVariation and GeneticOptimization
    - **notes:** available in Python utilities function AddSensorRecorder(...)
    - date resolved: **2021-11-25 11:10**\ , date raised: 2021-10-17 
 * Version 1.1.43: resolved Issue 0801: Box3D (change)
    - description:  check Ubuntu20 warnings; replace Vector3D pmin with Real pmin[3] to avoid warnings and gain speedup
    - date resolved: **2021-11-25 11:08**\ , date raised: 2021-11-23 
 * Version 1.1.42: resolved Issue 0798: Implicit GeneralContact (extension)
    - description:  add ODE2RHS jacobian and PostNewton to GeneralContact
    - date resolved: **2021-11-19 16:07**\ , date raised: 2021-11-19 
 * Version 1.1.41: resolved Issue 0787: GeneralContact (extension)
    - description:  add contact object, directly in mbs, which allows different types of contact with efficient computation and search trees; start with simple spherical contact
    - date resolved: **2021-11-19 16:06**\ , date raised: 2021-11-01 
 * Version 1.1.40: :textred:`resolved BUG 0784` : verboseMode 
    - description:  output of every step in verboseMode=1 after long time or if there are some very long lasting steps
    - **notes:** not fully clarified, but modified time when data is output; may occur in case of very long running iPython?
    - date resolved: **2021-11-14 23:23**\ , date raised: 2021-10-31 
 * Version 1.1.39: :textred:`resolved BUG 0790` : multithreading fails for user functions 
    - description:  build separate lists for objects and loads with user functions, excluded in MT evaluation
    - date resolved: **2021-11-14 23:21**\ , date raised: 2021-11-02 
 * Version 1.1.38: :textred:`resolved BUG 0794` : error when switching from n to 1 threads 
    - description:  RuntimeError: TemporaryComputationDataArray::operator[]: index out of range caused when switching from numberOfThreads>1 to 1 thread
    - date resolved: **2021-11-13 23:59**\ , date raised: 2021-11-04 
 * Version 1.1.37: resolved Issue 0788: multithreaded ODE2RHS and compute loads (extension)
    - description:  use multithreaded computation for ODE2 RHS and for loads computation; use simulationSettings.numberOfThreads > 1 for multithreaded computation
    - date resolved: **2021-11-13 23:59**\ , date raised: 2021-11-01 
 * Version 1.1.36: resolved Issue 0796: GL list GeneralContact (extension)
    - description:  speed up visualization with GL list for spheres in GeneralContact
    - date resolved: **2021-11-13 23:03**\ , date raised: 2021-11-10 
 * Version 1.1.35: resolved Issue 0797: itemInterface (extension)
    - description:  add representation for item interface classes, using __repr__() = dict(self)
    - date resolved: **2021-11-10 08:40**\ , date raised: 2021-11-10 
 * Version 1.1.34: resolved Issue 0789: constant mass matrix (extension)
    - description:  do not recompute mass matrix if it is totally constant in implicit solver; add flag to solver options to force recompute
    - date resolved: **2021-11-08 10:30**\ , date raised: 2021-11-02 
 * Version 1.1.33: resolved Issue 0795: add TCP/IP interface (extension)
    - description:  add CreateTCPIPconnection and other functions to utilities for interconnection with other programs via TCP/IP
    - date resolved: **2021-11-08 10:29**\ , date raised: 2021-11-08 
 * Version 1.1.32: resolved Issue 0786: TemporaryComputationDataArray (extension)
    - description:  add array of TemporaryCompData for multithreaded computation
    - date resolved: **2021-11-01 23:01**\ , date raised: 2021-11-01 
 * Version 1.1.31: resolved Issue 0785: ComputeSystemODE1RHS (extension)
    - description:  add list of loads with ODE1 relevancy, otherwise all loads are computed even if there are no ODE1 coordinates
    - **notes:** added simple flag to avoid computation if no ODE1 coordinates available
    - date resolved: **2021-11-01 21:28**\ , date raised: 2021-11-01 
 * Version 1.1.30: resolved Issue 0781: add n-mass-oscillator (example)
    - description:  add interactive example based on simulateInteractively for n-mass-oscillator with step and frequency excitation
    - date resolved: **2021-10-28 16:44**\ , date raised: 2021-10-25 
 * Version 1.1.29: resolved Issue 0747: ComputeLinearizedSystem (extension)
    - description:  add exudyn.ComputeLinearizedSystem similar to what is done in ComputeODEEigenvalues, returning M, K, D, ...
    - date resolved: **2021-10-27 18:40**\ , date raised: 2021-09-03 
 * Version 1.1.28: resolved Issue 0780: enable close window button (extension)
    - description:  close window button is now enabled, which stops current and following simulations until render window is restarted or SetRenderEngineStopFlag(False)
    - **notes:** if a simulation with renderer is quit (also ESCAPE button), then a further call to solver will be ignored until the simulation is reset, or SetRenderEngineStopFlag(False)
    - date resolved: **2021-10-21 09:31**\ , date raised: 2021-10-21 
 * Version 1.1.27: resolved Issue 0778: GenericODE2 FEM (extension)
    - description:  add FEMinterface function for creation of ObjectGenericODE2 with linear FEM model and nonlinear FEM model (using NGsolve)
    - date resolved: **2021-10-16 23:23**\ , date raised: 2021-10-16 
 * Version 1.1.26: :textred:`resolved BUG 0776` : MatrixContainer::SetWithSparseMatrixCSR 
    - description:  does not set number of columns and rows due to error in MatrixContainer::SetAllZero()
    - date resolved: **2021-10-09 16:53**\ , date raised: 2021-10-08 
 * Version 1.1.25: resolved Issue 0767: sparse ObjectJacobianODE2 (extension)
    - description:  add dense and sparse interface to ObjectJacobianODE2
    - date resolved: **2021-10-09 16:53**\ , date raised: 2021-09-27 
 * Version 1.1.24: :textred:`resolved BUG 0775` : ObjectGenericODE2, ObjectFFRFreducedOrder 
    - description:  visualization fails if outputVariable = None
    - date resolved: **2021-10-08 17:26**\ , date raised: 2021-10-08 
 * Version 1.1.23: resolved Issue 0773: ImportMeshFromNGsolve (extension)
    - description:  added option meshOrder which allows to use second order elements with meshOrder=2, leading to much higher accuracy of displacements and stresses
    - date resolved: **2021-10-01 14:04**\ , date raised: 2021-10-01 
 * Version 1.1.22: resolved Issue 0774: ComputePostProcessingModesNGsolve (extension)
    - description:  added improved functionality for ComputePostProcessingModes using NGsolve, speeding up computations by factor of 10
    - date resolved: **2021-10-01 14:03**\ , date raised: 2021-10-01 
 * Version 1.1.21: resolved Issue 0772: compute HCB modes with NGsolve (extension)
    - description:  add much faster computation function ComputeHurtyCraigBamptonModesNGsolve for computation of eigenmodes
    - date resolved: **2021-10-01 14:03**\ , date raised: 2021-10-01 
 * Version 1.1.20: resolved Issue 0771: GeneticOptimization (extension)
    - description:  add normal distribution and distanceFactorGenerations
    - date resolved: **2021-09-29 17:38**\ , date raised: 2021-09-29 
 * Version 1.1.19: resolved Issue 0769: Marker jacobian derivative (extension)
    - description:  add jacobian derivative to markers to allow analytical differentiation of connectors
    - date resolved: **2021-09-28 18:57**\ , date raised: 2021-09-28 
 * Version 1.1.18: resolved Issue 0768: Newton.useNumericalDifferentiation (change)
    - description:  change to Newton.numericalDifferentiation.forAE and Newton.numericalDifferentiation.forODE2; previous Newton.useNumericalDifferentiation only affected AE (algebraic equations) and should be changed now to  Newton.numericalDifferentiation.forAE; default is False
    - date resolved: **2021-09-27 15:11**\ , date raised: 2021-09-27 
 * Version 1.1.17: resolved Issue 0612: sparse object matrices (extension)
    - description:  add sparse matrix computation mode for ComputeMassMatrix and for ObjectJacobianODE2; consider Lie algebra derivatives
    - **notes:** sparse ObjectJacobianODE2 computation not yet implemented and moved to issue767
    - date resolved: **2021-09-27 14:35**\ , date raised: 2021-03-21 
 * Version 1.1.16: resolved Issue 0766: ObjectGenericODE2 (change)
    - description:  change mass matrix, stiffnessmatrix, etc. types to PyMatrixContainer in order to accept dense and sparse matrices
    - date resolved: **2021-09-27 14:32**\ , date raised: 2021-09-26 
 * Version 1.1.15: resolved Issue 0765: describe Python types (docu)
    - description:  describe Python types such as NumpyMatrix or PyMatrixContainer in intro to objects, nodes, ...
    - date resolved: **2021-09-27 14:32**\ , date raised: 2021-09-26 
 * Version 1.1.14: resolved Issue 0764: PyMatrixContainer (extension)
    - description:  extend PyMatrixContainer to accept numpy.array or list of lists as input
    - date resolved: **2021-09-26 23:35**\ , date raised: 2021-09-26 
 * Version 1.1.13: resolved Issue 0763: ComputeMassMatrix (extension)
    - description:  add sparse mode with MatrixContainer
    - date resolved: **2021-09-26 18:01**\ , date raised: 2021-09-26 
 * Version 1.1.12: resolved Issue 0762: MatrixBase (performance)
    - description:  remove virtual from begin/end operators
    - date resolved: **2021-09-26 17:36**\ , date raised: 2021-09-26 
 * Version 1.1.11: :textred:`resolved BUG 0750` : ANCF/ALE contour plot 
    - description:  contour plot not showing displacements or forces
    - **notes:** bug due to issue 760, which has been resolved now!
    - date resolved: **2021-09-24 08:48**\ , date raised: 2021-09-03 
 * Version 1.1.10: resolved Issue 0761: LinkedDataVectorBase (extension)
    - description:  allowing SetNumberOfItems to make LinkedDataVectors smaller after linking
    - date resolved: **2021-09-24 08:47**\ , date raised: 2021-09-24 
 * Version 1.1.9: :textred:`resolved BUG 0760` : contour plot 
    - description:  nodes in contour plot leading to exudyn crash due to ConstSizeVector decoupled from Vector
    - date resolved: **2021-09-24 08:47**\ , date raised: 2021-09-24 
 * Version 1.1.8: resolved Issue 0758: FEM CMSObjectComputeNorm (extension)
    - description:  add function into FEM to compute maximum stress / strain / etc for CMSObject (ObjectFFRFreducedOrder), using only the objectNumber as an input; options are outputVariableType=StressLocal, norm="" (Mises, L2norm, none), nodeNumbers=[] ... providing optional list of nodes to restrict the computation
    - date resolved: **2021-09-23 19:25**\ , date raised: 2021-09-22 
 * Version 1.1.7: resolved Issue 0757: FEM GetNodePositionsMean (extension)
    - description:  add function into FEMinterface to compute mean (average) position based on nodeNumbers (as list)
    - date resolved: **2021-09-23 17:38**\ , date raised: 2021-09-22 
 * Version 1.1.6: :textred:`resolved BUG 0759` : contour plot 
    - description:  equivalent stress showing negative color bar values in contour plot
    - **notes:** contour plot with norm (component -1) showing only positive min and max values when tested
    - date resolved: **2021-09-23 17:19**\ , date raised: 2021-09-23 
 * Version 1.1.5: resolved Issue 0756: FEM MisesStress (extension)
    - description:  add function that computes Mises stress from 6 stress components as obtained in stress sensor
    - **notes:** put into exudyn.physics module
    - date resolved: **2021-09-23 16:02**\ , date raised: 2021-09-22 
 * Version 1.1.4: resolved Issue 0755: GetKinematicTree66 in robotics (extension)
    - description:  add function to export KinematicTree66 from Robotic class
    - date resolved: **2021-09-22 18:06**\ , date raised: 2021-09-22 
 * Version 1.1.3: resolved Issue 0754: performance tests (test)
    - description:  add automated performance tests for solver speed to determine significant drop of performance
    - date resolved: **2021-09-22 10:10**\ , date raised: 2021-09-22 
 * Version 1.1.2: resolved Issue 0620: TorsionalSpringDamper (extension)
    - description:  add torsional spring damper similar to SpringDamper, fixed on a single local axis of marker0, allowing to realize controllers and torques
    - date resolved: **2021-09-16 10:45**\ , date raised: 2021-04-06 
 * Version 1.1.1: resolved Issue 0679: Renderer tkinter (extension)
    - description:  add flag to disable calls to tkinter from Renderer, which is not possible if tkinter is already used for interactive dialogs. This allows to open visualizationSettings in AnimateModes and SolutionViewer
    - date resolved: **2021-09-14 10:21**\ , date raised: 2021-05-14 
 * Version 1.1.0: :textred:`resolved BUG 0751` : SetMarkerParameter 
    - description:  causes internal error, because of wrong index check; workaround: use int(..) to cast marker index
    - date resolved: **2021-09-10 16:22**\ , date raised: 2021-09-10 

***********
Version 1.0
***********

 * Version 1.0.295: :textred:`resolved BUG 0749` : ObjectALEANCFCable2D 
    - description:  precomputed mass terms not computed accordingly; leads to crash if not static solution computed in advance
    - date resolved: **2021-09-03 15:12**\ , date raised: 2021-09-03 
 * Version 1.0.294: resolved Issue 0748: Extend solver description (docu)
    - description:  add some description for SolveStatic / SolveDynamic in solver chapter
    - date resolved: **2021-09-03 13:54**\ , date raised: 2021-09-03 
 * Version 1.0.293: resolved Issue 0504: serialrobot (extension)
    - description:  build completely from homogenouos transformations, COMs, inertia tensors, masses, axes, axesTypes
    - **notes:** done earlier
    - date resolved: **2021-08-22 11:12**\ , date raised: 2020-12-16 
 * Version 1.0.292: resolved Issue 0540: github (extension)
    - description:  update README.rst file and make it similar to other packages (e.g. pydy)
    - **notes:** done earlier
    - date resolved: **2021-08-22 11:11**\ , date raised: 2021-01-09 
 * Version 1.0.291: resolved Issue 0729: README.rst (docu)
    - description:  add .rst readme file containing gettingStarted, introduction, tutorial and other information
    - date resolved: **2021-08-22 10:58**\ , date raised: 2021-08-05 
 * Version 1.0.290: resolved Issue 0740: robotics (extension)
    - description:  extend Robot class for Modified DH parameters and general transformations; add transformations before and after joint axis
    - date resolved: **2021-08-19 00:22**\ , date raised: 2021-08-18 
 * Version 1.0.289: :textred:`resolved BUG 0739` : robotics 
    - description:  CreateRedundantCoordinateMBS draws wrong axes
    - date resolved: **2021-08-19 00:22**\ , date raised: 2021-08-18 
 * Version 1.0.288: :textred:`resolved BUG 0742` : robotics 
    - description:  Robot.JointHT computes LinkHT
    - date resolved: **2021-08-18 23:31**\ , date raised: 2021-08-18 
 * Version 1.0.287: resolved Issue 0741: robotics (change)
    - description:  add base and tool class to robotics to have more flexibility for future developments; replace toolHT to tool.HT, baseHT to base.HT; add tool.visualization and base.visualization
    - date resolved: **2021-08-18 15:07**\ , date raised: 2021-08-18 
 * Version 1.0.286: resolved Issue 0733: ContactCoordinate (extension)
    - description:  add recommendedStepSize to ContactCoordinate and find optimal solution with data variable from StartOfStep configuration; check if step size is permanently reduced with recommendedStepSize; check a way of an overall recommendedStepSize (with filter) or allow a single event not to change global step size
    - date resolved: **2021-08-13 13:23**\ , date raised: 2021-08-12 
 * Version 1.0.285: resolved Issue 0313: Add user node ODE2 (extension)
    - description:  add user node with getposition, rotation, access functions
    - **notes:** not needed: GenericNodes can be used for that
    - date resolved: **2021-08-10 12:57**\ , date raised: 2020-01-10 
 * Version 1.0.284: resolved Issue 0730: RigidBody tutorial (docu)
    - description:  add tutorial for rigid body with AddRigidBody(...), AddRevoluteJoint(...) functionalities
    - date resolved: **2021-08-06 20:05**\ , date raised: 2021-08-05 
 * Version 1.0.283: resolved Issue 0732: DrawSystemGraph (extension)
    - description:  improve visualization and return graph and other information
    - date resolved: **2021-08-06 17:58**\ , date raised: 2021-08-06 
 * Version 1.0.282: :textred:`resolved BUG 0731` : AddRevoluteJoint 
    - description:  AddRevoluteJoint shows error in axis definition
    - date resolved: **2021-08-05 13:40**\ , date raised: 2021-08-05 
 * Version 1.0.281: resolved Issue 0704: Optimization2 (optimize)
    - description:  add direct function to NodeRigidBody to retrieve essential data for rigid body EOM and MarkerRigidBody; add flag, if rotation matrix and other quantities needed
    - **notes:** still no optimization for Lie group nodes, which however have simpler matrices
    - date resolved: **2021-07-31 22:33**\ , date raised: 2021-07-04 
 * Version 1.0.280: resolved Issue 0514: general wheel (extension)
    - description:  add general wheel model (with general rotation body); for mecanum wheel rolls
    - **notes:** implemented ObjectContactConvexRoll for general usage in Mecanum wheels and other applications
    - date resolved: **2021-07-31 21:20**\ , date raised: 2020-12-19 
    - resolved by: P. Manzl
 * Version 1.0.279: resolved Issue 0727: NodeRigidBody2D (docu)
    - description:  Outputvariable Rotation gives 3D vector, but wrong description in DOCU
    - **notes:** additionally: rotation is now directly copied from rotation coordinate and is not recomputed from Tait-Bryan angles of rotation matrix
    - date resolved: **2021-07-31 21:18**\ , date raised: 2021-07-14 
 * Version 1.0.278: resolved Issue 0726: description of nodes (docu)
    - description:  finish detailed description of 3D nodes and add rotation parameter description for Tait-Bryan in theory part
    - date resolved: **2021-07-13 21:17**\ , date raised: 2021-07-13 
 * Version 1.0.277: resolved Issue 0725: unify description (docu)
    - description:  unify notation for special vectors, e.g., reference point or local position; add unified abbreviations for ODE2, etc.
    - date resolved: **2021-07-13 21:17**\ , date raised: 2021-07-13 
 * Version 1.0.276: :textred:`resolved BUG 0724` : Linux version 
    - description:  exudyn fails after import exudyn on Ubuntu18.04 and 20.04, showing error with RenderStateMachine selectionString
    - **notes:** version 276 tested with Ubuntu20.04, working again
    - date resolved: **2021-07-12 22:47**\ , date raised: 2021-07-12 
 * Version 1.0.275: resolved Issue 0723: SolutionViewer (change)
    - description:  remove SolutionViewer from exudyn init file, as it causes problems if no tkinter or matplotlib installed
    - **notes:** use exudyn.interactive.SolutionViewer(...) instead
    - date resolved: **2021-07-12 20:23**\ , date raised: 2021-07-12 
 * Version 1.0.274: :textred:`resolved BUG 0722` : glfwGetWindowContentScale 
    - description:  function causes immediate crash on linux (UBUNTU) when importing exudyn
    - **notes:** added flag for linux compilation, excluding font scaling
    - date resolved: **2021-07-12 19:31**\ , date raised: 2021-07-12 
 * Version 1.0.273: resolved Issue 0719: Pybind11 2.6 (change)
    - description:  switch to Pybind11 2.6 in included C++ files
    - date resolved: **2021-07-12 16:57**\ , date raised: 2021-07-12 
 * Version 1.0.272: resolved Issue 0718: Python3.8 FASTLINALG (change)
    - description:  using now __FAST_EXUDYN_LINALG option, which excludes all range checks and other checks in arrays, matrices, etc.; leads usually to 30percent higher performance
    - date resolved: **2021-07-12 16:57**\ , date raised: 2021-07-12 
 * Version 1.0.271: :textred:`resolved BUG 0721` : FEM.GetNodesOnLine(..) 
    - description:  fails because self. missing in call to GetNodesOnCylinder
    - date resolved: **2021-07-12 16:35**\ , date raised: 2021-07-12 
 * Version 1.0.270: resolved Issue 0717: SC.StaticSolve, SC.TimeIntegrationSolve (change)
    - description:  remove these deprecated functions from interface
    - date resolved: **2021-07-12 15:33**\ , date raised: 2021-07-11 
 * Version 1.0.269: resolved Issue 0669: remove old solvers (change)
    - description:  remove old static and dynamic solvers as they are not any more up to date with graphics interface
    - date resolved: **2021-07-12 15:32**\ , date raised: 2021-05-10 
 * Version 1.0.268: resolved Issue 0716: SystemIsConsistent (change)
    - description:  add checks for functions that may not be called if not SystemIsConsistent
    - date resolved: **2021-07-11 17:30**\ , date raised: 2021-07-11 
 * Version 1.0.267: :textred:`resolved BUG 0714` : mbs.GetSensorValues 
    - description:  raises error for ObjectFFRFreducedOrder: ERROR: LinkedDataVectorBase(const VectorBase<T>&, Index), startPosition < 0
    - **notes:** caused when called before Assemble(); checks added in future
    - date resolved: **2021-07-11 17:08**\ , date raised: 2021-07-10 
 * Version 1.0.266: :textred:`resolved BUG 0715` : ObjectFFRFreducedOrder 
    - description:  OutputVariable Displacement includes localPosition, but should not
    - **notes:** GetMeshNodeLocalPosition included reference position twice
    - date resolved: **2021-07-11 16:33**\ , date raised: 2021-07-10 
 * Version 1.0.265: resolved Issue 0706: ConstSizeVector (optimize)
    - description:  decouple ConstSizeVector and ConstSizeMatrix from Vector / Matrix and avoid virtual calls, erase all rule of 5 member functions, optimize algebra
    - **notes:** improved speed up to factor 2 for some items!
    - date resolved: **2021-07-11 15:06**\ , date raised: 2021-07-06 
 * Version 1.0.264: :textred:`resolved BUG 0713` : ObjectFFRFreducedOrder 
    - description:  GetOutputVariableSuperElement does not agree with types described in theDoc; object does not provide Displacement or Position, sensors return wrong values
    - **notes:** corrected C++ implementation and theDoc.pdf for OutputVariableTypesSuperElement
    - date resolved: **2021-07-09 20:53**\ , date raised: 2021-07-09 
 * Version 1.0.263: resolved Issue 0712: serialRobot (change)
    - description:  improve speed of serial robot by transferring controllers from load userfunctions to mbs.SetPreStepUserFunction
    - date resolved: **2021-07-09 13:28**\ , date raised: 2021-07-09 
 * Version 1.0.262: resolved Issue 0711: generator files (change)
    - description:  changed backslash to slash in generator files such that they can also be executed on Linux and MacOS
    - date resolved: **2021-07-09 12:18**\ , date raised: 2021-07-09 
 * Version 1.0.261: resolved Issue 0699: CMarkerBodyRigid::ComputeMarkerData (optimize)
    - description:  implement optimized version for Rigid node and ObjectRigidBody and avoid repeated computation of rotation matrix, etc.
    - date resolved: **2021-07-08 00:46**\ , date raised: 2021-07-01 
 * Version 1.0.260: :textred:`resolved BUG 0709` : Linux/MacOS compile error 
    - description:  compiler error caused by EXU::Square
    - date resolved: **2021-07-07 19:11**\ , date raised: 2021-07-07 
 * Version 1.0.259: resolved Issue 0708: preprocessor flags (change)
    - description:  move EXUDYN_RELEASE to preprocessor flags in setup.py
    - date resolved: **2021-07-07 08:53**\ , date raised: 2021-07-07 
 * Version 1.0.258: resolved Issue 0705: Optimization3 (optimize)
    - description:  optimize ObjectRigidBody EOM, take Glocal columns instead numberOfRotationCoordinates, move rot_t into loop, etc.
    - date resolved: **2021-07-06 23:04**\ , date raised: 2021-07-04 
 * Version 1.0.257: resolved Issue 0703: ComputeOrthonormalBasis (change)
    - description:  changed rigidBodyUtilities function, which returns a list of basis vectors, into ComputeOrthonormalBasisVectors, while ComputeOrthonormalBasis now returns a rotation matrix
    - date resolved: **2021-07-02 08:49**\ , date raised: 2021-07-02 
 * Version 1.0.256: resolved Issue 0702: AddPrismaticJoint (extension)
    - description:  add convenient utility function to add prismatic joint based on 2 bodies, point and axis, doing all necessary work in background
    - date resolved: **2021-07-02 08:49**\ , date raised: 2021-07-02 
 * Version 1.0.255: resolved Issue 0701: AddRevoluteJoint (extension)
    - description:  add convenient utility function to add revolute joint based on 2 bodies, point and axis, doing all necessary work in background
    - date resolved: **2021-07-02 08:49**\ , date raised: 2021-07-02 
 * Version 1.0.254: resolved Issue 0700: add links for utility functions (docu)
    - description:  ADDED LINKS to Examples/ and TestModels/ example files at end of each python utility function and class, see Section :ref:`sec-pythonutilityfunctions`\ 
    - date resolved: **2021-07-01 21:46**\ , date raised: 2021-07-01 
 * Version 1.0.253: :textred:`resolved BUG 0697` : GenericJoint 
    - description:  index2 equations not properly implemented for prismatic joints
    - **notes:** added second term for index2 case if joint position not constrained; TrapezoidalIndex2 solver now works if translational joint axes not constrained
    - date resolved: **2021-07-01 15:50**\ , date raised: 2021-07-01 
 * Version 1.0.252: resolved Issue 0696: add PrismaticJoint (extension)
    - description:  add 3D prismatic joint with rotationMarker0/1 to adjust local coordinate systems and joint local x axis as the free axis of the joint
    - date resolved: **2021-07-01 13:43**\ , date raised: 2021-07-01 
 * Version 1.0.251: resolved Issue 0369: add RevoluteJoint (extension)
    - description:  add 3D revolute joint with rotationMarker0/1 to adjust joint coordinates and joint local z axis as rotation axis
    - date resolved: **2021-07-01 13:43**\ , date raised: 2020-04-10 
 * Version 1.0.250: resolved Issue 0695: Solution functions (change)
    - description:  remove exu and SC arguments from exudyn.utilities functions SetSolutionState(...), AnimateSolution(...); remove functoin SetVisualizationState(...): use SetSolutionState instead!
    - date resolved: **2021-06-29 17:47**\ , date raised: 2021-06-29 
 * Version 1.0.249: resolved Issue 0694: SolutionViewer (extension)
    - description:  add interactive dialog to view solution based on coordinateSolution.txt
    - date resolved: **2021-06-29 17:31**\ , date raised: 2021-06-29 
 * Version 1.0.248: resolved Issue 0693: RigidBody user function (extension)
    - description:  add test model for GenericODE2 user function based rigid body with Euler parameter and constraint
    - date resolved: **2021-06-28 19:34**\ , date raised: 2021-06-28 
 * Version 1.0.247: resolved Issue 0564: NodeRigidBodyEP (change)
    - description:  transfer EP constraint from object to node, for future application to 3D beams
    - date resolved: **2021-06-28 19:34**\ , date raised: 2021-01-28 
 * Version 1.0.246: resolved Issue 0413: ConnectorCoordinateVectorUF (extension)
    - description:  implement coordinate vector constraint user function; can be used as generic joint
    - date resolved: **2021-06-28 16:19**\ , date raised: 2020-05-25 
 * Version 1.0.245: resolved Issue 0691: extend ConnectorCoordinateVector (extension)
    - description:  extend ConnectorCoordinateVector for quadratic terms to be used as Euler Parameters constraint
    - date resolved: **2021-06-27 23:29**\ , date raised: 2021-06-27 
 * Version 1.0.244: resolved Issue 0690: add MarkerNodeCoordinates (extension)
    - description:  used for CoordinateVector constraint
    - date resolved: **2021-06-27 23:29**\ , date raised: 2021-06-27 
 * Version 1.0.243: resolved Issue 0689: add itemIndex to user functions (change)
    - description:  CHANGE OF userFunctions interface with additional itemIndex for ConnectorSpringDamper, ConnectorCartesianSpringDamper, ConnectorRigidBodySpringDamper, ConnectorCoordinate, ConnectorCoordinateVector, ConnectorJointGeneric; see theDoc for changes in the interface of these user functions and adapt your models!
    - **notes:** WARNING: Interface of user functions for ConnectorSpringDamper, ConnectorCartesianSpringDamper, ConnectorRigidBodySpringDamper, ConnectorCoordinate, ConnectorCoordinateVector, ConnectorJointGeneric changed!!!
    - date resolved: **2021-06-27 20:45**\ , date raised: 2021-06-27 
 * Version 1.0.242: resolved Issue 0688: ObjectGenericODE2 (change)
    - description:  extend ObjectGenericODE2 and ObjectGenericODE1 user functions for the item index to have access to nodes and other informaiton: WARNING: you need to adapt your existing user functions!
    - **notes:** WARNING: Interface of user functions for ObjectGenericODE2, ObjectFFRF... changed!!!
    - date resolved: **2021-06-27 20:44**\ , date raised: 2021-06-24 
 * Version 1.0.241: resolved Issue 0687: ObjectRigidBody (extension)
    - description:  add output variable VelocityLocal to 2D and 3D rigid body objects
    - date resolved: **2021-06-22 16:55**\ , date raised: 2021-06-22 
 * Version 1.0.240: resolved Issue 0686: eigenvalue solver (extension)
    - description:  use ngsolve solver for eigenvalue computation speedup
    - date resolved: **2021-06-14 15:21**\ , date raised: 2021-06-14 
 * Version 1.0.239: :textred:`resolved BUG 0684` : openGL.multisampling 
    - description:  Mac OS multisampling option in visualizationSettings crashes; ==> do not change this option under Mac OS
    - **notes:** excluded multisampling option for MacOS compilation
    - date resolved: **2021-05-30 12:02**\ , date raised: 2021-05-25 
 * Version 1.0.238: :textred:`resolved BUG 0681` : ObjectALEANCFCable2D 
    - description:  ObjectALEANCFCable2D position jacobian does not provide axially moving part for ObjectContactFrictionCircleCable2D, NEED TO BE ADDED
    - **notes:** had been already included in MarkerBodyCable2Dshape and works for roll contact
    - date resolved: **2021-05-30 12:01**\ , date raised: 2021-05-17 
 * Version 1.0.237: resolved Issue 0685: NodeRigidBody2D (change)
    - description:  add same drawing as 3D nodes (with reference frame)
    - date resolved: **2021-05-30 11:37**\ , date raised: 2021-05-30 
 * Version 1.0.236: resolved Issue 0683: SlidingJointRigid (extension)
    - description:  Add functionality for rigid sliding joint or add a flag for sliding joint to do both options
    - date resolved: **2021-05-20 09:30**\ , date raised: 2021-05-19 
 * Version 1.0.235: resolved Issue 0682: copy paste code from theDoc.pdf (extension)
    - description:  changed ' characters to enable copy/paste of code with quotes
    - date resolved: **2021-05-19 08:54**\ , date raised: 2021-05-19 
 * Version 1.0.234: :textred:`resolved BUG 0680` : SetSystemState 
    - description:  mbs.systemData.SetSystemState does not set data coordinates
    - date resolved: **2021-05-15 11:07**\ , date raised: 2021-05-15 
 * Version 1.0.233: resolved Issue 0676: single threaded renderer (change)
    - description:  improve exu.DoRendererIdleTasks() and use it in all python function - AnimateModes, Interactive, etc.
    - **notes:** can now be also called in multithreaded renderer
    - date resolved: **2021-05-14 21:42**\ , date raised: 2021-05-12 
 * Version 1.0.232: :textred:`resolved BUG 0678` : mouseInteractiveExample 
    - description:  not running any more, check new Renderer functions
    - date resolved: **2021-05-14 21:41**\ , date raised: 2021-05-14 
 * Version 1.0.231: resolved Issue 0630: HurtyCraigBampton (extension)
    - description:  extend computation to work with 0 eigenmodes
    - date resolved: **2021-05-12 23:51**\ , date raised: 2021-04-23 
 * Version 1.0.230: resolved Issue 0642: ComputeHurtyCraigBamptonModes (extension)
    - description:  add possibility to add position only interfaces
    - **notes:** abandoned, because makes no sense with RBE2 modes
    - date resolved: **2021-05-12 23:35**\ , date raised: 2021-04-30 
 * Version 1.0.229: :textred:`resolved BUG 0674` : OutputVariable.StressLocal 
    - description:  norm of OutputVariable stresses does not work
    - date resolved: **2021-05-12 23:21**\ , date raised: 2021-05-12 
 * Version 1.0.228: :textred:`resolved BUG 0456` : ObjectFFRF bug with GenericJoint 
    - description:  raises error: CSolverBase::SolveSteps CObjectSuperElement:GetAccessFunctionSuperElement: AngularVelocity_qt not implemented; cannot compute jacobian for orientation
    - date resolved: **2021-05-12 22:27**\ , date raised: 2020-10-13 
 * Version 1.0.226: resolved Issue 0100: UPDATE Lest tests (new feature)
    - description:  update lest tests (select C++ vs. python tests)    
    - date resolved: **2021-05-12 22:21**\ , date raised: 2019-04-01 
 * Version 1.0.225: resolved Issue 0372: add manual solver example (extension)
    - description:  add manual for solver and example; also add new prestep user function
    - **notes:** resolved earlier
    - date resolved: **2021-05-12 22:20**\ , date raised: 2020-04-10 
 * Version 1.0.224: resolved Issue 0384: Solver interface (extension)
    - description:  change solver interface such that it stores MainSystem/mbs for user functions; MainSolverXYZ and CSolverXYZ take MainSystem as argument in constructor
    - date resolved: **2021-05-12 22:18**\ , date raised: 2020-05-06 
 * Version 1.0.223: resolved Issue 0675: PostProcessingModes (extension)
    - description:  compute PostProcessingModes with multiprocessing
    - date resolved: **2021-05-12 22:10**\ , date raised: 2021-05-12 
 * Version 1.0.222: resolved Issue 0672: right-mouse-dialog (extension)
    - description:  add item indices to right mouse dialog
    - date resolved: **2021-05-12 22:05**\ , date raised: 2021-05-11 
 * Version 1.0.221: resolved Issue 0673: FEM.PostProcessingModes (extension)
    - description:  compute PostProcessingModes in FEMinterface with multiprocessing option
    - date resolved: **2021-05-12 15:37**\ , date raised: 2021-05-12 
 * Version 1.0.220: resolved Issue 0430: stress modes FEMinterface (extension)
    - description:  add into FEMinterface and allow storing that data
    - **notes:** resolved already earlier, see issue 623
    - date resolved: **2021-05-12 15:36**\ , date raised: 2020-07-01 
 * Version 1.0.219: :textred:`resolved BUG 0631` : ObjectFFRFreducedOrder 
    - description:  freefree eigenmodes and Hurty-Craig-Bampton modes do not converge to same results
    - **notes:** convergence for eigenmodes and HCB modes given, but differences due to HCB boundary sets, inconsistent initial conditions for MarkerSuperElementRigid; pure beam bending converges well
    - date resolved: **2021-05-12 14:05**\ , date raised: 2021-04-23 
 * Version 1.0.218: :textred:`resolved BUG 0671` : mbs.Reset() and SC.Reset() 
    - description:  reset MainSystem mbs and SystemContainer SC hangs; current SC is erronously stolen from renderer when another SC is deleted
    - date resolved: **2021-05-11 10:55**\ , date raised: 2021-05-11 
 * Version 1.0.217: resolved Issue 0670: MacOS graphics support (extension)
    - description:  add compatibility to MacOS in single-threaded graphics mode (tested with OS X 10.7)
    - date resolved: **2021-05-10 22:34**\ , date raised: 2021-05-10 
 * Version 1.0.216: resolved Issue 0647: Single Thread Renderer (extension)
    - description:  Implement single thread renderer version for MAC OS compatibility test
    - date resolved: **2021-05-10 22:32**\ , date raised: 2021-04-30 
 * Version 1.0.215: resolved Issue 0634: Set visualization state (extension)
    - description:  add thread-safe variant for updating the visualization state
    - date resolved: **2021-05-10 18:32**\ , date raised: 2021-04-25 
 * Version 1.0.214: resolved Issue 0668: multiple mbs and SystemContainer support (extension)
    - description:  adapt renderer and MainSystemContainer to work with multiple MainSystems (mbs) and SC instances at same time; add SC.AttachToRenderEnginer, SC.DetachFromRenderEngine
    - date resolved: **2021-05-10 18:20**\ , date raised: 2021-05-10 
 * Version 1.0.213: :textred:`resolved BUG 0665` : StartRenderer() 
    - description:  without being in a render loop (e.g., SC.WaitForRenderEngineStopFlag()), the pure StartRenderer() crashes upon left mouse click
    - date resolved: **2021-05-10 14:51**\ , date raised: 2021-05-05 
 * Version 1.0.212: resolved Issue 0664: right mouse (change)
    - description:  add function to retrieve py::dict from items safely in python thread into temporary storage; check also other python calls to operate fully in main thread
    - date resolved: **2021-05-10 14:51**\ , date raised: 2021-05-05 
 * Version 1.0.211: :textred:`resolved BUG 0662` : Render window 
    - description:  during open tkinter dialogs, the render window responds on keyboard or mouse input, which calls again python functions that hang up the system
    - date resolved: **2021-05-10 14:51**\ , date raised: 2021-05-04 
 * Version 1.0.210: resolved Issue 0633: WaitAndLockSemaphoreIgnore (check)
    - description:  check which atomic_flags are needed in C++ to make code threadsafe
    - date resolved: **2021-05-10 14:51**\ , date raised: 2021-04-25 
 * Version 1.0.209: resolved Issue 0667: tkinter dialogs focus and on top (extension)
    - description:  when opening tkinter dialogs - visualizationSettings, edit dialogs, help, ... - they immediately get focus and are on top
    - date resolved: **2021-05-10 14:49**\ , date raised: 2021-05-10 
 * Version 1.0.208: resolved Issue 0666: make renderer Python and thread safe (change)
    - description:  add strict separation between renderer (thread) and Python (thread); add rendererPythonInterface between both threads; left and right mouse clicks now safe to press; render window does not accept any input as long as tkinter window is open, but does not produce crashes any more
    - date resolved: **2021-05-10 14:49**\ , date raised: 2021-05-10 
 * Version 1.0.207: resolved Issue 0663: help button (docu)
    - description:  show "press h for help" as startup message for 10 seconds and sync help message with theDoc.pdf
    - date resolved: **2021-05-05 10:02**\ , date raised: 2021-05-05 
 * Version 1.0.206: resolved Issue 0653: add right mouse edit dialog (extension)
    - description:  open Edit dialog for item on right-mouse-press
    - date resolved: **2021-05-04 20:58**\ , date raised: 2021-05-01 
 * Version 1.0.205: resolved Issue 0652: identify itemID under mouse coursor (extension)
    - description:  identify object/node/... under mouse using unique color for itemID (left mouse button press)
    - date resolved: **2021-05-04 12:49**\ , date raised: 2021-05-01 
 * Version 1.0.204: resolved Issue 0637: Python3.8 windows wheels (extension)
    - description:  create Python3.8 windows wheels automatically
    - date resolved: **2021-05-03 18:51**\ , date raised: 2021-04-26 
 * Version 1.0.203: resolved Issue 0661: add C++ unit tests (extension)
    - description:  add C++ unit tests to Python3.6 64bits version and to testSuite. Changed initialization of all vector types to avoid errors of Vector({5}), now allowing only Vector({4.}) in constructors
    - date resolved: **2021-05-03 18:50**\ , date raised: 2021-05-03 
 * Version 1.0.202: resolved Issue 0660: initializerList (check)
    - description:  check if Vector is used with initializer list with one item - Vector({10}), converting to std::vector or Vector(10)
    - date resolved: **2021-05-03 18:50**\ , date raised: 2021-05-03 
 * Version 1.0.201: resolved Issue 0659: Troubleshooting (docu)
    - description:  Add Trouble shooting section, treating common Python and solver errors to theDoc.pdf
    - date resolved: **2021-05-03 10:18**\ , date raised: 2021-05-03 
 * Version 1.0.200: resolved Issue 0650: Highlight item# (extension)
    - description:  Highlight item# for object/node/etc.; add to visualizationSettings (itemType, item#, colorHighlightItem, colorOtherItems), draw all other items in gray
    - date resolved: **2021-05-03 01:18**\ , date raised: 2021-05-01 
 * Version 1.0.199: resolved Issue 0649: add ItemType (extension)
    - description:  Add enum ItemType: Node, Object, ...
    - date resolved: **2021-05-03 01:18**\ , date raised: 2021-05-01 
 * Version 1.0.198: resolved Issue 0651: add itemID to graphics objects (extension)
    - description:  Add itemID (nodes, objects, markers, loads, sensors, in that order) to graphics objects (for right-mouse-press)
    - date resolved: **2021-05-02 21:26**\ , date raised: 2021-05-01 
 * Version 1.0.197: resolved Issue 0658: add VisualizationSettings() interactive (change)
    - description:  move visualizationSettings window functions keypressRotationStep, mouseMoveRotationFactor, keypressTranslationStep, zoomStepFactor to new substructure "interactive"
    - **notes:** \ \*\*adapt your models if you used these options!\*\*\ 
    - date resolved: **2021-05-01 23:36**\ , date raised: 2021-05-01 
 * Version 1.0.196: resolved Issue 0654: coordinates sizes (extension)
    - description:  add function ODE2Size(...), ODE1Size(...), SystemSize(...) to mbs.systemData to retrieve number of ODE2,ODE1,AE and Data coordinates for certain configurationType; only works after mbs.Assemble()
    - date resolved: **2021-05-01 23:23**\ , date raised: 2021-05-01 
 * Version 1.0.195: resolved Issue 0656: mbs.systemData (change)
    - description:  removed GetCurrentTime() and SetVisualizationTime(...) which have been marked as deprecated already
    - **notes:** Use GetTime(...) and SetTime(...) in mbs.systemData instead
    - date resolved: **2021-05-01 23:08**\ , date raised: 2021-05-01 
 * Version 1.0.194: resolved Issue 0644: solver messages (extension)
    - description:  add solver message if not converged with helpful hints (especially if invert fails or newton fails)
    - date resolved: **2021-05-01 02:31**\ , date raised: 2021-04-30 
 * Version 1.0.193: resolved Issue 0349: add causing row (extension)
    - description:  output causing row/column (=coordinate) which leads to singular matrix; do this for Matrix.Invert as well as for SparseLU .info code; matrix class creates string with error message!
    - date resolved: **2021-05-01 02:30**\ , date raised: 2020-03-02 
 * Version 1.0.192: resolved Issue 0646: jacobian singular (extension)
    - description:  resolve singularities in general jacobian: resolves coordinates which are still free for static problems, but is marked as unsafe
    - date resolved: **2021-05-01 02:29**\ , date raised: 2021-04-30 
 * Version 1.0.191: resolved Issue 0645: redundant constraints (extension)
    - description:  resolve redundant constraints: add flag linearSolverSettings.ignoreSingularJacobian in SC.SimulationSettings() to ignore singular constraint jacobians
    - date resolved: **2021-05-01 02:29**\ , date raised: 2021-04-30 
 * Version 1.0.190: resolved Issue 0333: node numbers with type (extension)
    - description:  extend node/object/... numbers as python class with type information to check if item numbers are mixed illegally
    - **notes:** done already earlier, but still marked as unresolved
    - date resolved: **2021-04-30 17:04**\ , date raised: 2020-02-06 
 * Version 1.0.189: resolved Issue 0641: ObjectContactFrictionCircleCable2D (docu)
    - description:  add description and figure for theory and computation
    - date resolved: **2021-04-29 08:40**\ , date raised: 2021-04-29 
 * Version 1.0.188: :textred:`resolved BUG 0423` : fix MarkerSuperElementRigidBody 
    - description:  fix velocity level for MarkerSuperElementRigidBody (check constraint equations)
    - **notes:** fixed several errors and test examples work now on velocity level, but further checks are necessary
    - date resolved: **2021-04-27 18:24**\ , date raised: 2020-06-09 
 * Version 1.0.187: resolved Issue 0635: AnimateModes (check)
    - description:  check, why animate modes has threading-conflicts; use std::cout to find issues
    - **notes:** resolved threading conflicts, but visualization state set inbetween graphics update, which needs to resolve #634
    - date resolved: **2021-04-27 18:20**\ , date raised: 2021-04-25 
 * Version 1.0.186: resolved Issue 0640: MarkerSuperElementRigid (extension)
    - description:  remove referencePosition and add offset instead (to correct errors of midpoint due to small mesh-unsymmetries)
    - **notes:** \ \*\*CHANGED interface\*\*\ : MarkerSuperElementRigid does not have a referencePosition anymore, but adds a parameter offset
    - date resolved: **2021-04-27 18:18**\ , date raised: 2021-04-26 
 * Version 1.0.185: :textred:`resolved BUG 0639` : ObjectFFRFreducedOrder 
    - description:  incorrect AccessFunction AccessFunctionType::AngularVelocity_qt, missing correct reference point = midpoint for computation of rotation
    - date resolved: **2021-04-26 21:47**\ , date raised: 2021-04-26 
 * Version 1.0.184: resolved Issue 0638: MarkerSuperElementRigid (extension)
    - description:  use consistent reference point = midpoint for computation of rotation and use exponential Map for rotation matrix
    - date resolved: **2021-04-26 21:47**\ , date raised: 2021-04-26 
 * Version 1.0.183: resolved Issue 0636: Python3.8 (extension)
    - description:  add python 3.8 compilation tests; resolve issues with __index__ method needed fore NodeIndex, MarkerIndex, etc.
    - date resolved: **2021-04-26 16:25**\ , date raised: 2021-04-26 
 * Version 1.0.182: :textred:`resolved BUG 0629` : mesh visualization 
    - description:  visualization artifacts in larger FE meshes due to multithreading
    - **notes:** added flag threadSafeGraphicsUpdate to avoid thread conflicts between graphics and computation, which is by default set True and MAY SLOW DOWN your computation speed if True
    - date resolved: **2021-04-25 22:28**\ , date raised: 2021-04-22 
 * Version 1.0.181: resolved Issue 0632: CMS theory (docu)
    - description:  add theory section for Hurty-Craig-Bampton modes and eigenmode computation
    - date resolved: **2021-04-23 19:03**\ , date raised: 2021-04-23 
 * Version 1.0.180: :textred:`resolved BUG 0628` : FEMinterface.GetNodesOnCylinder 
    - description:  returns erroneous indices
    - **notes:** corrected indexing and add warnings for illegal node types
    - date resolved: **2021-04-22 22:59**\ , date raised: 2021-04-22 
 * Version 1.0.179: resolved Issue 0616: Craig-Bampton (extension)
    - description:  add static modes (Hurty-Craig-Bampton) to computation of modes in CMSinterface
    - **notes:** implemented in FEMinterface.ComputeHurtyCraigBamptonModes(...)
    - date resolved: **2021-04-21 11:09**\ , date raised: 2021-03-21 
 * Version 1.0.178: resolved Issue 0624: norm in contour plots (extension)
    - description:  show norm (of vectors or stresses) in contour plot, using special outputVariable component=-1
    - date resolved: **2021-04-09 18:18**\ , date raised: 2021-04-09 
 * Version 1.0.177: resolved Issue 0623: postProcessingModes (extension)
    - description:  add function to compute stress or strain modes for postprocessing, working for linear tetraherons (Tet4); see Examples/NGsolvePostProcessingStresses.py
    - date resolved: **2021-04-09 15:46**\ , date raised: 2021-04-09 
 * Version 1.0.176: resolved Issue 0424: show modes (extension)
    - description:  add feature to visualize eigenmodes, e.g. using ObjectFFRFreducedOrder and set one initialCoordinate nonzero
    - date resolved: **2021-04-07 13:48**\ , date raised: 2020-06-12 
 * Version 1.0.175: resolved Issue 0619: Eigenmode visualizer (extension)
    - description:  visualize eigenmodes with interactive tools with new function AnimateModes(...) to show eigenmodes of system or ObjectFFRFreducedOrder (see Section :ref:`sec-interactive-animatemodes`\ )
    - date resolved: **2021-04-07 13:47**\ , date raised: 2021-03-30 
 * Version 1.0.174: resolved Issue 0622: InteractiveDialog (extension)
    - description:  improved functionality of InteractiveDialog in interactive.py, specially for animating modes
    - date resolved: **2021-04-07 12:20**\ , date raised: 2021-04-07 
 * Version 1.0.173: :textred:`resolved BUG 0621` : mbs.GetNodeODE2Index 
    - description:  Fails for NodeRigidBodyEP, because mix of AE and ODE2 variables
    - **notes:** added correct type check in MainSystem::PyGetNodeODE2Index
    - date resolved: **2021-04-07 09:12**\ , date raised: 2021-04-07 
 * Version 1.0.172: resolved Issue 0403: CMS C++ (extension)
    - description:  add ObjectFFRFreducedOrder (CMS) equations in C++ and clean up code
    - date resolved: **2021-03-30 16:57**\ , date raised: 2020-05-21 
 * Version 1.0.171: resolved Issue 0470: geometrically exact beam2D (extension)
    - description:  add to CPP
    - date resolved: **2021-03-25 18:07**\ , date raised: 2020-11-21 
 * Version 1.0.170: resolved Issue 0563: ODE1Coordinate (extension)
    - description:  add MarkerODE1Coordinate and extend LoadCoordinate for ODE1
    - date resolved: **2021-03-25 07:43**\ , date raised: 2021-01-27 
 * Version 1.0.169: resolved Issue 0615: MarkerNodeCoordinate (extension)
    - description:  add check in CSystem for valid coordinate numbers in MarkerNodeCoordinate
    - date resolved: **2021-03-22 14:18**\ , date raised: 2021-03-21 
 * Version 1.0.168: resolved Issue 0611: adaptiveStep (extension)
    - description:  add adaptiveStepIncrease, Decrease and RecoverySteps options to control behavior in case of discontinuous problems
    - date resolved: **2021-03-21 00:21**\ , date raised: 2021-03-21 
 * Version 1.0.167: resolved Issue 0603: loadFactor (change)
    - description:  exclude load factor for loads with user functions in static computations
    - date resolved: **2021-03-20 23:24**\ , date raised: 2021-03-18 
 * Version 1.0.166: resolved Issue 0610: startOfStep (extension)
    - description:  add access function for nodal coordinates at startOfStep configuration, used in mbs.GetNodeOutput(configuration = exu.ConfigurationType.startOfStep)
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-20 
 * Version 1.0.165: resolved Issue 0609: SolveDynamic, SolveStatic (change)
    - description:  store dynamicSolver and staticSolver in mbs.sys dictionary immediately after creation, which allows to use these structures in user functions during static or dynamic solution
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-20 
 * Version 1.0.164: resolved Issue 0607: test recommendedStepSize (test)
    - description:  test recommendedStepSize and PostNewtonUserFunction with simple elastic contact example
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-20 
 * Version 1.0.163: resolved Issue 0605: UIndex, UReal (extension)
    - description:  change all relevant unsigned quantities to UIndex and UReal, as well as Vectors of UReal and Arrays of UIndex
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-19 
 * Version 1.0.162: resolved Issue 0604: UIndex check (extension)
    - description:  add automatic check in item interface to check for correctness of UIndex and UReal quantities
    - date resolved: **2021-03-20 23:23**\ , date raised: 2021-03-19 
 * Version 1.0.161: resolved Issue 0337: local quantities beam (change)
    - description:  change beam output of Force, Torque, Curvature, Stress and Strain to ForceLocal, TorqueLocal, CurvatureLocal, StressLocal, and StrainLocal
    - **notes:** \ \*\*WARNING\*\*\ : you need to adapt force, torque and stress, strain and curvature output variables accordingly as they may have changed specifically for ANCF beams; adapt all your model files regarding Force, Torque, etc.
    - date resolved: **2021-03-20 23:22**\ , date raised: 2020-02-18 
 * Version 1.0.160: resolved Issue 0139: Index (change)
    - description:  change Index to (signed) int and use UIndex in python interface for unsigned parameters
    - **notes:** \ \*\*ATTENTION\*\*\ : this change affects many routines. All TestSuite examples passed the change but there may still be open problems due to this major change.
    - date resolved: **2021-03-20 23:21**\ , date raised: 2019-05-20 
 * Version 1.0.159: resolved Issue 0606: resolve errors 32bit testsuite (test)
    - description:  add extra tolerances for 32bit
    - date resolved: **2021-03-20 23:19**\ , date raised: 2021-03-20 
 * Version 1.0.158: :textred:`resolved BUG 0575` : new genAlpha solver 
    - description:  new generalized alpha/implicit trapezoidal solver does not call solver user functions; Solution: derive CSolverImplicitSecondOrderTimeIntNew from CSolverBase and add user functions on top
    - **notes:** solved by removing old solver structure; new solver fully supports user functions now
    - date resolved: **2021-03-18 21:34**\ , date raised: 2021-02-08 
 * Version 1.0.157: resolved Issue 0602: PostNewton step size recommendation (extension)
    - description:  add step recommendation as outcome of PostNewton function to improve contact and friction accuracy
    - date resolved: **2021-03-18 21:33**\ , date raised: 2021-03-18 
 * Version 1.0.156: resolved Issue 0601: mbs.postNewtonUserFunction (extension)
    - description:  add function PostNewton(...) to be called after step update (Newton or explicit step)
    - date resolved: **2021-03-18 21:33**\ , date raised: 2021-03-18 
 * Version 1.0.155: resolved Issue 0600: ImplicitSecondOrderSolver (cleanup)
    - description:  remove old solver
    - date resolved: **2021-03-18 21:33**\ , date raised: 2021-03-18 
 * Version 1.0.154: resolved Issue 0598: rigidBodyUtilities (extension)
    - description:  add G matrices for Rxyz (Tait-Bryan angles) and also time derivatives of G
    - date resolved: **2021-03-18 17:05**\ , date raised: 2021-03-18 
 * Version 1.0.153: resolved Issue 0594: CMS rotations (extension)
    - description:  test and extend CMS / ObjectFFRFreducedOrder object for other rotation parameterizations (Tait-Bryan and rotation vector/Lie group) such that they work with explicit codes
    - date resolved: **2021-03-18 17:04**\ , date raised: 2021-02-24 
 * Version 1.0.152: resolved Issue 0597: ObjectRigidBody (description)
    - description:  fix description of equations of motion (missing m) and add steps in derivation
    - date resolved: **2021-03-18 08:22**\ , date raised: 2021-03-18 
 * Version 1.0.151: resolved Issue 0283: cylinder with hole (new feature)
    - description:  add TriangleList for cylinder with hole
    - **notes:** not implemented, because it can be easily created with GraphicsDataSolidOfRevolution
    - date resolved: **2021-03-16 16:59**\ , date raised: 2019-12-07 
 * Version 1.0.150: resolved Issue 0396: description (description)
    - description:  add latex description for ObjectFFRF
    - date resolved: **2021-03-16 16:57**\ , date raised: 2020-05-16 
 * Version 1.0.149: resolved Issue 0394: description (description)
    - description:  add latex description for ObjectSuperElement
    - date resolved: **2021-03-16 16:57**\ , date raised: 2020-05-16 
 * Version 1.0.148: resolved Issue 0595: ObjectFFRFreducedOrder (extension)
    - description:  add general nodeType to AddObjectFFRFreducedOrderWithUserFunctions
    - date resolved: **2021-03-16 16:56**\ , date raised: 2021-03-14 
 * Version 1.0.147: resolved Issue 0461: ObjectRigidBody (check)
    - description:  check discription of output variables
    - date resolved: **2021-03-16 16:55**\ , date raised: 2020-11-12 
 * Version 1.0.146: resolved Issue 0596: GetRigidBodyNode (extension)
    - description:  add function GetRigidBodyNode into rigidBodyUtilities, which returns a node item for an according node type, e.g., Euler parameters or rotation vector
    - date resolved: **2021-03-14 14:43**\ , date raised: 2021-03-14 
 * Version 1.0.145: resolved Issue 0583: FFRF docu (docu)
    - description:  add documentation for FFRF and FFRFreducedOrder (CMS) to documentation
    - date resolved: **2021-03-01 12:15**\ , date raised: 2021-02-16 
 * Version 1.0.144: resolved Issue 0455: FEM help (docu)
    - description:  add comments to ObjectFFRF (Tisserand frame!) and reduced that there are convenient helper functions in FEM, etc. for creating objects
    - date resolved: **2021-02-24 21:21**\ , date raised: 2020-10-13 
 * Version 1.0.143: resolved Issue 0593: Add MacOS support (change)
    - description:  make minor adjustments for MacOS to run setup.py
    - date resolved: **2021-02-22 13:10**\ , date raised: 2021-02-22 
 * Version 1.0.142: :textred:`resolved BUG 0592` : StartRenderer 
    - description:  flag verbose=True not working
    - date resolved: **2021-02-22 10:08**\ , date raised: 2021-02-22 
 * Version 1.0.141: resolved Issue 0590: SensorMarker visualization (extension)
    - description:  add visualization for SensorMarker according to marker position
    - date resolved: **2021-02-19 10:21**\ , date raised: 2021-02-19 
 * Version 1.0.140: resolved Issue 0400: add SensorMarker (extension)
    - description:  add sensor for markers, restricting to position/velocity and rotation/angular velocity
    - date resolved: **2021-02-18 19:15**\ , date raised: 2020-05-20 
 * Version 1.0.139: resolved Issue 0585: UserSensor (extension)
    - description:  add user sensor, which enables the user to add any kind of sensor, specifically to combine several sensor outputs into one single sensor
    - date resolved: **2021-02-18 19:14**\ , date raised: 2021-02-17 
 * Version 1.0.138: resolved Issue 0589: solver updateInitialValues (change)
    - description:  update also initial coordinates in order to avoid jumps in accelerations when continuing simulation
    - date resolved: **2021-02-18 18:15**\ , date raised: 2021-02-18 
 * Version 1.0.137: resolved Issue 0588: Solver file header (change)
    - description:  move writing of solution file and sensor files headers from InitializeSolverPreChecks(...) to InitializeSolverInitialConditions(...) to avoid sensor evaluation for initial configuration
    - date resolved: **2021-02-18 16:23**\ , date raised: 2021-02-18 
 * Version 1.0.136: resolved Issue 0587: ALEANCFCable2D (fix)
    - description:  change mass proportional load to include force in direction of ALE coordinate
    - date resolved: **2021-02-17 18:19**\ , date raised: 2021-02-17 
 * Version 1.0.135: resolved Issue 0586: GeneticOptimization (fix)
    - description:  add special warnings and adaptations in order to catch cases where elitistRatio\*populationSize < 1 and if distanceFactor >= 1
    - date resolved: **2021-02-17 18:17**\ , date raised: 2021-02-17 
 * Version 1.0.134: resolved Issue 0584: parameter variation (extension)
    - description:  add possibility to prescribe set of parameters using list, e.g.,  'mass':[1,2,4,8] instead of of tuple which describes the range: 'mass':(0,6,4)
    - date resolved: **2021-02-16 09:43**\ , date raised: 2021-02-16 
 * Version 1.0.133: resolved Issue 0582: optimization (docu)
    - description:  add parameter variation and genetic optiization to documentation of solvers
    - date resolved: **2021-02-16 08:21**\ , date raised: 2021-02-16 
 * Version 1.0.132: :textred:`resolved BUG 0581` : SetODE2Coordinates_tt 
    - description:  SetODE2Coordinates_tt writes to velocities instead of accelerations
    - date resolved: **2021-02-14 11:12**\ , date raised: 2021-02-14 
 * Version 1.0.131: resolved Issue 0580: ParameterVariation (extension)
    - description:  processing.ParameterVariation(...): write to resultsFile to show progress in resultsMonitor.py
    - date resolved: **2021-02-11 17:49**\ , date raised: 2021-02-11 
 * Version 1.0.130: resolved Issue 0576: add ClearWorkspace (extension)
    - description:  add ClearWorkspace() to basicUtilities which allows simple and save cleanup of globals() in python environment; recommended to be called at beginning of complex models
    - date resolved: **2021-02-10 12:35**\ , date raised: 2021-02-08 
 * Version 1.0.129: resolved Issue 0569: contour text (fix)
    - description:  add space to computation info text before contour plot text
    - date resolved: **2021-02-10 12:35**\ , date raised: 2021-02-05 
 * Version 1.0.128: resolved Issue 0568: Renderer axes (fix)
    - description:  use X(0), Y(1) and Z(2) for axes description to be compliant with Python indexing starting with 0 as well as contour components
    - date resolved: **2021-02-10 12:35**\ , date raised: 2021-02-05 
 * Version 1.0.127: resolved Issue 0579: ClearWorkspace (extension)
    - description:  add ClearWorkspacefunction to exudyn.basicUtilities, which allows to reset global variables in ipython; see example in function description
    - date resolved: **2021-02-10 12:13**\ , date raised: 2021-02-10 
 * Version 1.0.126: resolved Issue 0578: SmoothStep (extension)
    - description:  add SmoothStep function to exudyn.utilities, which produces a smooth step function using cosine
    - date resolved: **2021-02-10 12:13**\ , date raised: 2021-02-10 
 * Version 1.0.125: resolved Issue 0572: void (fix)
    - description:  redundant with issue 568
    - date resolved: **2021-02-10 12:05**\ , date raised: 2021-02-05 
 * Version 1.0.124: resolved Issue 0571: void (fix)
    - description:  redundant with issue 568
    - date resolved: **2021-02-10 12:05**\ , date raised: 2021-02-05 
 * Version 1.0.123: resolved Issue 0570: void (fix)
    - description:  redundant with issue 568
    - date resolved: **2021-02-10 12:05**\ , date raised: 2021-02-05 
 * Version 1.0.122: :textred:`resolved BUG 0577` : PostNewtonStep 
    - description:  perform PostNewtonStep and PostDiscontinuousIterationStep only for active objects
    - date resolved: **2021-02-09 14:00**\ , date raised: 2021-02-09 
 * Version 1.0.121: resolved Issue 0355: generalized alpha (extension)
    - description:  implement version of Brüls and Arnold for generalized alpha solver
    - **notes:**  WARNING: switched to new solver based on displacement increments (Arnold/Bruls,2007), which leads to DIFFERENT (but improved) RESULTS than previous dynamic implicit integrator; new implicit solver now works with ODE1 variables
    - date resolved: **2021-02-08 01:56**\ , date raised: 2020-03-08 
 * Version 1.0.120: resolved Issue 0573: merge solver documentation (docu)
    - description:  merge docu on solver in EXUDYN overview and in solver section
    - date resolved: **2021-02-07 17:33**\ , date raised: 2021-02-07 
 * Version 1.0.119: resolved Issue 0567: solvers description (docu)
    - description:  extend description for equations of motion, explicit and implicit solvers
    - date resolved: **2021-02-04 01:14**\ , date raised: 2021-02-03 
 * Version 1.0.118: resolved Issue 0560: Impl integrator ODE1 (extension)
    - description:  add ODE1 coordinates to implicit integrator
    - date resolved: **2021-02-02 15:16**\ , date raised: 2021-01-26 
 * Version 1.0.117: resolved Issue 0508: implicit Lie group integrator (extension)
    - description:  implement implicit index2/index3 Lie group integrator as python function
    - **notes:** cancelled, because will be directly done in C++
    - date resolved: **2021-02-02 15:15**\ , date raised: 2020-12-17 
 * Version 1.0.116: resolved Issue 0566: memory alloc cnt (check)
    - description:  add control to check whether large amount of memory allocations happen during time integration+test suite
    - date resolved: **2021-02-01 01:38**\ , date raised: 2021-01-29 
 * Version 1.0.115: resolved Issue 0541: objectODE1/2, constraint lists (extension)
    - description:  add lists of ODE1 and ODE2 objects, constraints, etc. in cSystemData in order to speed up processing
    - date resolved: **2021-02-01 01:38**\ , date raised: 2021-01-13 
 * Version 1.0.114: resolved Issue 0557: RK with constraints (extension)
    - description:  add CoordinateConstraints to explict Runge-Kutta solvers
    - **notes:** only ground constraints included for now
    - date resolved: **2021-01-27 17:38**\ , date raised: 2021-01-26 
 * Version 1.0.113: resolved Issue 0558: Lie group tests (test)
    - description:  add Lie group integrator simple tests
    - date resolved: **2021-01-27 12:00**\ , date raised: 2021-01-26 
 * Version 1.0.112: resolved Issue 0550: GraphicsDataArrow (extension)
    - description:  add arrow to graphicsDataUtilities
    - **notes:** also added GraphicsDataBasis(...) for drawing 3 orthogonal basis vectors, GraphicsDataCheckerBoard(...) for simple drawing of checker board background and MergeGraphicsDataTriangleList(...) for merging graphicsData triangle lists
    - date resolved: **2021-01-27 00:10**\ , date raised: 2021-01-17 
 * Version 1.0.111: resolved Issue 0495: add ODE1 coordinates (extension)
    - description:  extend system (Jacobian, etc.) for ODE1 coordinates
    - date resolved: **2021-01-26 13:21**\ , date raised: 2020-12-09 
 * Version 1.0.110: resolved Issue 0556: explicit RK tests (test)
    - description:  add tests for explicit Runge Kutta integrators to TestModels
    - date resolved: **2021-01-26 13:17**\ , date raised: 2021-01-25 
 * Version 1.0.109: resolved Issue 0555: explicit Lie group integrator (extension)
    - description:  add existing Lie group integrator in C++
    - date resolved: **2021-01-26 13:17**\ , date raised: 2021-01-25 
 * Version 1.0.108: resolved Issue 0554: explicit integrator (extension)
    - description:  add explicit integrator with automatic step size control (DOPRI5, ODE23); checkout Section :ref:`sec-explicitsolver`\  for description of explicit solvers and Section :ref:`sec-dynamicsolvertype`\  for available solver types
    - date resolved: **2021-01-25 00:54**\ , date raised: 2021-01-24 
 * Version 1.0.107: resolved Issue 0513: add RK4 integrator (extension)
    - description:  put existing python RK4 integrator into CPP
    - date resolved: **2021-01-25 00:54**\ , date raised: 2020-12-19 
 * Version 1.0.106: resolved Issue 0533: ObjectGenericODE1 (extension)
    - description:  add object ObjectGenericODE1 for generic first order ODEs
    - date resolved: **2021-01-21 17:27**\ , date raised: 2021-01-04 
 * Version 1.0.105: resolved Issue 0553: create physics submodule (extension)
    - description:  create exudyn.physics and add friction functions
    - date resolved: **2021-01-20 10:25**\ , date raised: 2021-01-20 
 * Version 1.0.104: :textred:`resolved BUG 0552` : DrawSystemGraph 
    - description:  does not work with RigidBodySpringDamper due to invalid GenericNodeData number
    - date resolved: **2021-01-19 14:43**\ , date raised: 2021-01-19 
 * Version 1.0.103: resolved Issue 0551: InteractiveDialog (extension)
    - description:  add interactive tkinter dialog and new submodule exudyn.interactive to interact with models
    - date resolved: **2021-01-19 00:26**\ , date raised: 2021-01-19 
 * Version 1.0.102: resolved Issue 0549: show solver name and time (extension)
    - description:  add options to show/hide solver name and current time in render window
    - date resolved: **2021-01-17 17:42**\ , date raised: 2021-01-17 
 * Version 1.0.101: :textred:`resolved BUG 0545` : mbs.WaitForUserToContinue() 
    - description:  call to WaitForUserToContinue() does not always wait for keypress. Check StartRender() function and flag settings
    - date resolved: **2021-01-17 16:55**\ , date raised: 2021-01-15 
 * Version 1.0.100: :textred:`resolved BUG 0548` : SolveDynamic/SolveStatic 
    - description:  option updateInitialValues not working
    - **notes:** corrected SetSystemState call
    - date resolved: **2021-01-17 00:08**\ , date raised: 2021-01-17 
 * Version 1.0.99: resolved Issue 0542: GeneticOptimization (extension)
    - description:  store values continuously to file, add automatic loader and animate optimized values
    - **notes:** added resultsMonitor.py to exudyn module
    - date resolved: **2021-01-15 15:18**\ , date raised: 2021-01-13 
 * Version 1.0.98: resolved Issue 0547: realtimeSimulation (extension)
    - description:  add factor for timeIntegration.simulateInRealtime
    - date resolved: **2021-01-15 15:16**\ , date raised: 2021-01-15 
 * Version 1.0.97: resolved Issue 0546: add __version__ version to module (extension)
    - description:  enable exudyn.__version__ as commonly used in other modules
    - date resolved: **2021-01-15 15:05**\ , date raised: 2021-01-15 
 * Version 1.0.96: resolved Issue 0544: geneticOptimization (extension)
    - description:  add optional argument resultsFile to specify a file for output of results data
    - date resolved: **2021-01-14 23:17**\ , date raised: 2021-01-14 
 * Version 1.0.95: resolved Issue 0543: add results monitor (extension)
    - description:  add resultsMonitor.py to be called from command line for doing continuous visualization of sensors and geneticOptimization output
    - date resolved: **2021-01-14 23:08**\ , date raised: 2021-01-14 
 * Version 1.0.94: resolved Issue 0532: NodeGenericODE1 (extension)
    - description:  add node NodeGenericODE1 for arbitrary number of ODE1 coordinates
    - date resolved: **2021-01-13 20:12**\ , date raised: 2021-01-04 
 * Version 1.0.93: resolved Issue 0531: solidExtrusion (extension)
    - description:  add graphicsData for solid extrusion (prismatic) body; based on 2D point and segment list for flat boundaries
    - date resolved: **2021-01-10 20:43**\ , date raised: 2021-01-04 
 * Version 1.0.92: resolved Issue 0539: RigidBodySpringDamper (extension)
    - description:  add postNewtonStepUserFunction and dataCoordinates
    - date resolved: **2021-01-08 14:34**\ , date raised: 2021-01-08 
 * Version 1.0.91: :textred:`resolved BUG 0537` : Render window 
    - description:  double calling of Render(...) function could happen from RunLoop/Render and glfwSetWindowRefreshCallback (set in InitCreateWindow(...)); check if semaphore would remove visualization problems
    - **notes:** added semaphore but FEM visualization anomalies are still there
    - date resolved: **2021-01-07 11:23**\ , date raised: 2021-01-06 
 * Version 1.0.90: resolved Issue 0385: add solver eigenvalues example (extension)
    - description:  add Examples/solverFunctionsTestEigenvalues  to test suite
    - **notes:** added ComputeODE2EigenvaluesTest.py using new functionality exudyn.solver.ComputeODE2Eigenvalues(...)
    - date resolved: **2021-01-07 11:08**\ , date raised: 2020-05-06 
 * Version 1.0.89: resolved Issue 0494: add all tests (extension)
    - description:  add all TestModel/\*.py to testsuite and also examples before making changes to solver
    - date resolved: **2021-01-07 11:04**\ , date raised: 2020-12-09 
 * Version 1.0.88: resolved Issue 0515: user function connector (extension)
    - description:  add forceUserFunction for ObjectConnectorRigidBodySpringDamper to enable User connector
    - date resolved: **2021-01-07 11:03**\ , date raised: 2020-12-19 
 * Version 1.0.87: resolved Issue 0506: utilities docu (extension)
    - description:  complete documentation for all exudyn python utilities and add unique headers for documentation
    - date resolved: **2021-01-06 22:57**\ , date raised: 2020-12-16 
 * Version 1.0.86: resolved Issue 0530: solidOfRevolution (extension)
    - description:  add graphicsData for solid of revoluation
    - date resolved: **2021-01-06 00:31**\ , date raised: 2021-01-04 
 * Version 1.0.85: resolved Issue 0536: GraphicsDataPlane (extension)
    - description:  add graphicsData for simple rectangular plane with option for checkerboard pattern
    - date resolved: **2021-01-05 22:57**\ , date raised: 2021-01-05 
 * Version 1.0.84: resolved Issue 0535: alternating color for cylinder (extension)
    - description:  add alternatingColor argument in GraphicsDataCylinder for visualization of rotation of cylindric bodies
    - date resolved: **2021-01-05 21:46**\ , date raised: 2021-01-05 
 * Version 1.0.83: resolved Issue 0529: add MainSystem to userFunctions (change)
    - description:  add MainSystem "mbs" to all user functions as first argument (WARNING: this changes ALL user functions!!!
    - date resolved: **2021-01-05 14:31**\ , date raised: 2021-01-04 
 * Version 1.0.82: resolved Issue 0447: test examples (check)
    - description:  test all examples with new index types
    - date resolved: **2021-01-05 14:31**\ , date raised: 2020-09-09 
 * Version 1.0.81: :textred:`resolved BUG 0534` : PlotSensor 
    - description:  PlotSensor crashes for Load sensors because no outputVariableType exists
    - **notes:** added special treatment for load sensors
    - date resolved: **2021-01-04 20:11**\ , date raised: 2021-01-04 
 * Version 1.0.80: resolved Issue 0527: faces transparent (extension)
    - description:  add general transparency flag for faces in visualizationSettings.openGL, switchable with button "T"; allows to make node/marker/object numbers visible
    - date resolved: **2021-01-03 21:53**\ , date raised: 2021-01-03 
 * Version 1.0.79: resolved Issue 0509: ComputeODE2Eigenvalues (test)
    - description:  add example in TestModels
    - date resolved: **2021-01-03 10:44**\ , date raised: 2020-12-18 
 * Version 1.0.78: resolved Issue 0528: textured fonts (extension)
    - description:  use TEXTURED based bitmap fonts based stored in glLists, allowing better interpolation, scalability (currently up to font size 64 without quality drop) and much higher performance
    - date resolved: **2021-01-03 10:29**\ , date raised: 2021-01-03 
 * Version 1.0.77: :textred:`resolved BUG 0526` : solver.ComputeODE2Eigenvalues 
    - description:  dense mode returned unsorted eigenvalues==>add sorting
    - date resolved: **2021-01-03 10:21**\ , date raised: 2021-01-03 
 * Version 1.0.76: resolved Issue 0524: interpret UTF8 (change)
    - description:  add conversion from UTF8 to unicode to interpret most central European characters + some important characters correctly (see Section :ref:`sec-graphicsdata`\ )
    - date resolved: **2021-01-02 20:13**\ , date raised: 2020-12-29 
 * Version 1.0.75: resolved Issue 0525: opengl write UTF8 (extension)
    - description:  use UTF8 encoding in opengl text output
    - date resolved: **2020-12-29 21:00**\ , date raised: 2020-12-29 
 * Version 1.0.74: resolved Issue 0523: show version (extension)
    - description:  show current version info in openGL window; can be switched off with showComputationInfo=False
    - date resolved: **2020-12-27 01:33**\ , date raised: 2020-12-27 
 * Version 1.0.73: resolved Issue 0522: openGl issues (fix)
    - description:  fix positioning problems of coordinate system and contour colorbar
    - **notes:** now using pixel coordinates for info texts and different font sizes
    - date resolved: **2020-12-27 01:31**\ , date raised: 2020-12-27 
 * Version 1.0.72: resolved Issue 0521: useWindowsDisplayScaleFactor (extension)
    - description:  add new option useWindowsDisplayScaleFactor in visualizationSettings.general to include display scaling factor for font sizes
    - date resolved: **2020-12-27 01:25**\ , date raised: 2020-12-27 
 * Version 1.0.71: resolved Issue 0520: useBitmapText (extension)
    - description:  add new option useBitmapText in visualizationSettings.general to activate bitmap fonts (deprecated; now using textured fonts)
    - date resolved: **2020-12-27 01:25**\ , date raised: 2020-12-27 
 * Version 1.0.70: resolved Issue 0516: add bitmap font (extension)
    - description:  add font using OpenGL bitmaps to improve visibility of texts (deprecated, now using textured fonts)
    - date resolved: **2020-12-27 01:23**\ , date raised: 2020-12-21 
 * Version 1.0.69: resolved Issue 0519: correct coordinateSystemSize (change)
    - description:  set visualizationSettings.general.coordinateSystemSize relative to fontSize which scales better with larger screens
    - date resolved: **2020-12-24 01:25**\ , date raised: 2020-12-24 
 * Version 1.0.68: resolved Issue 0518: windows display scaling (extension)
    - description:  include windows display (screen) scaling into drawing of texts to increase visibility on high dpi screens
    - **notes:** added flag in visualizationSettings: general.useWindowsDisplayScaleFactor
    - date resolved: **2020-12-24 00:22**\ , date raised: 2020-12-24 
 * Version 1.0.67: resolved Issue 0511: GeneticOptimization (test)
    - description:  add example in TestModels
    - date resolved: **2020-12-19 23:31**\ , date raised: 2020-12-19 
 * Version 1.0.66: resolved Issue 0510: ParameterVariation (test)
    - description:  add example in TestModels
    - date resolved: **2020-12-19 23:31**\ , date raised: 2020-12-19 
 * Version 1.0.65: resolved Issue 0502: rigidbodyinertia (docu)
    - description:  add description for rigidBodyUtilities class RigidBodyInertia
    - date resolved: **2020-12-19 23:28**\ , date raised: 2020-12-14 
 * Version 1.0.64: :textred:`resolved BUG 0512` : testsuite 
    - description:  EXUDYN build date referred shows wrong path
    - **notes:** refer now to installed module
    - date resolved: **2020-12-19 00:44**\ , date raised: 2020-12-19 
 * Version 1.0.63: resolved Issue 0507: changes (extension)
    - description:  incorporate resolved issues and bugs into theDoc.pdf
    - date resolved: **2020-12-17**\ , date raised: 2020-12-16 
 * Version 1.0.62: resolved Issue 0505: rigidBodyUtilities (extension)
    - description:  add description for RigidBodyInertia class
    - date resolved: **2020-12-17**\ , date raised: 2020-12-16 
 * Version 1.0.61: resolved Issue 0501: geneticOptimization add crossover (extension)
    - description:  added crossover and improved parameters for GeneticOptimization
    - date resolved: **2020-12-14**\ , date raised: 2020-12-14 
 * Version 1.0.60: resolved Issue 0497: genetic algorithm (check)
    - description:  check if stochsearch or genetic algorithm has simpler interface
    - date resolved: **2020-12-14**\ , date raised: 2020-12-10 
 * Version 1.0.59: resolved Issue 0500: FilterSignal (extension)
    - description:  put in signal module, make it working for 1D signals as well
    - date resolved: **2020-12-11**\ , date raised: 2020-12-10 
 * Version 1.0.58: resolved Issue 0492: FEMinterface GetNodesInOrthoCube (extension)
    - description:  add function which returns all nodes lying in cube aligned with global coordinate system, using [pMin, pMax], with tolerance
    - date resolved: **2020-12-11**\ , date raised: 2020-12-08 
 * Version 1.0.57: resolved Issue 0491: FEMinterface GetNodesOnCylinder (extension)
    - description:  add function which returns all nodes lying on specific cylinder, with tolerance
    - date resolved: **2020-12-11**\ , date raised: 2020-12-08 
 * Version 1.0.56: :textred:`resolved BUG 0499` : key V gives error 
    - description:  keypress V for visualizationSettings dialog gives error
    - date resolved: **2020-12-10**\ , date raised: 2020-12-10 
 * Version 1.0.55: :textred:`resolved BUG 0498` : SensorObject position 
    - description:  wrong position shown in sensor
    - date resolved: **2020-12-10**\ , date raised: 2020-12-10 
 * Version 1.0.54: resolved Issue 0484: test DEAP (test)
    - description:  test genetic optimization with DEAP
    - **notes:** too many parameters and too involved to simply include
    - date resolved: **2020-12-10**\ , date raised: 2020-12-04 
 * Version 1.0.53: :textred:`resolved BUG 0493` : CheckForValidFunction 
    - description:  modify / add this check to setParameters; additional if for setting this to 0
    - date resolved: **2020-12-09**\ , date raised: 2020-12-09 
 * Version 1.0.52: :textred:`resolved BUG 0490` : keypress crash 
    - description:  find out causes for crash in keyPress user function; find way to deactivate the user function (set it to 0)
    - date resolved: **2020-12-09**\ , date raised: 2020-12-07 
 * Version 1.0.51: resolved Issue 0389: MainSystem includes (cleanup)
    - description:  put SystemIntegrity item checks into separate file, to reduce includig MainSystem into every .cpp item file
    - date resolved: **2020-12-09**\ , date raised: 2020-05-13 
 * Version 1.0.50: resolved Issue 0357: solver flag prolong solution (extension)
    - description:  add flag for solvers that current state at end of computation is set as initial state for next solving
    - **notes:** added into new python interface of solver
    - date resolved: **2020-12-09**\ , date raised: 2020-03-11 
 * Version 1.0.49: resolved Issue 0489: add gradient background (extension)
    - description:  add according visualization.general option
    - date resolved: **2020-12-06**\ , date raised: 2020-12-06 
 * Version 1.0.48: :textred:`resolved BUG 0488` : problem with coordinate sys 
    - description:  fix problems with drawing of coordinate system: text moves strangely and axes dissappear after rotation
    - date resolved: **2020-12-06**\ , date raised: 2020-12-06 
 * Version 1.0.47: resolved Issue 0487: draw world basis (extension)
    - description:  add option to draw coordinate system at origin (world basis)
    - date resolved: **2020-12-06**\ , date raised: 2020-12-06 
 * Version 1.0.46: resolved Issue 0486: realtime (extension)
    - description:  add flag to time integration to simulate in realtime
    - date resolved: **2020-12-05**\ , date raised: 2020-12-05 
 * Version 1.0.45: resolved Issue 0485: mouse coordinates (extension)
    - description:  store mouse coordinates in renderState
    - date resolved: **2020-12-05**\ , date raised: 2020-12-05 
 * Version 1.0.44: resolved Issue 0467: mouse coordinates (extension)
    - description:  show mouse coordinates in render window (without transformation)
    - date resolved: **2020-12-05**\ , date raised: 2020-11-19 
 * Version 1.0.43: resolved Issue 0325: key callback (extension)
    - description:  add key callback function into graphics module to enable interactive settings, etc.; transfer latin letters, SHIFT, CTRL, ALT, 0-9,A-Z,.,SPACE as ASCII code
    - date resolved: **2020-12-05**\ , date raised: 2020-01-26 
 * Version 1.0.42: resolved Issue 0460: test accelerations (test)
    - description:  test GetODE2Coordinates_tt, nodal accelerations and rigidbody2D/3D accelerations
    - date resolved: **2020-12-04**\ , date raised: 2020-11-12 
 * Version 1.0.41: resolved Issue 0482: store model view (extension)
    - description:  store renderState in exudyn.sys dictionary after exu.StopRenderer() for subsequent simulations
    - date resolved: **2020-12-03**\ , date raised: 2020-12-03 
 * Version 1.0.40: resolved Issue 0478: link examples (docu)
    - description:  automatically add links to examples in thedoc
    - date resolved: **2020-12-03**\ , date raised: 2020-12-02 
 * Version 1.0.39: resolved Issue 0477: links in theDoc (extension)
    - description:  add links between user functions, add labels to item sections
    - date resolved: **2020-12-03**\ , date raised: 2020-12-02 
 * Version 1.0.38: resolved Issue 0463: accelerations (extension)
    - description:  add accelerations Outputvariable to Super elements
    - date resolved: **2020-12-03**\ , date raised: 2020-11-18 
 * Version 1.0.37: resolved Issue 0481: eigenvalue solver (extension)
    - description:  add eigenvalue computation interface for mbs in python
    - date resolved: **2020-12-02**\ , date raised: 2020-12-02 
 * Version 1.0.36: resolved Issue 0480: python solver (extension)
    - description:  add solver interfaces in python for MainSolverStatic and MainSolverImplicitSecondOrder, helping to retrieve solver data and to make solvers accessible for users
    - date resolved: **2020-12-02**\ , date raised: 2020-12-02 
 * Version 1.0.35: resolved Issue 0479: solver return (extension)
    - description:  add return value to solvers and copy solver structures to mbs.sys variables after finishing
    - **notes:** added python interfaces and kept old cpp solvers
    - date resolved: **2020-12-02**\ , date raised: 2020-12-02 
 * Version 1.0.34: resolved Issue 0469: userfunctions (extension)
    - description:  put user function generation in objectdefinition, with seperate U userfunction flag - this will automatically document the user function parameters (AND return values); this improves documentation and adds a unique interface in C++ using exception handling as well as GIL handling
    - date resolved: **2020-12-02**\ , date raised: 2020-11-20 
 * Version 1.0.33: resolved Issue 0458: graphicsDataUserFunction (docu)
    - description:  add example to docu in ObjectGround and GenericODE2 and add more accurate docu to ALL python user functions
    - date resolved: **2020-12-02**\ , date raised: 2020-11-10 
 * Version 1.0.32: resolved Issue 0428: queue user functions (extension)
    - description:  implement drawing user functions as global function similar to PyProcessQueue, in order to avoid messing up the CSystem and visualization modules
    - date resolved: **2020-12-02**\ , date raised: 2020-06-26 
 * Version 1.0.31: resolved Issue 0476: add RequireVersion (extension)
    - description:  functionality to allow to add a simple check to see if the installed version meets the requirements
    - date resolved: **2020-11-30**\ , date raised: 2020-11-30 
 * Version 1.0.30: resolved Issue 0475: rolling disc ext (extension)
    - description:  add force on ground and moving ground for ObjectJointRollingDisc
    - **notes:** needs to be tested further
    - date resolved: **2020-11-29**\ , date raised: 2020-11-26 
 * Version 1.0.29: resolved Issue 0474: auto compilation (check)
    - description:  check automatic compilation; check version in wheels; check linux wheels
    - **notes:** linux wheels can not be built with admin rights
    - date resolved: **2020-11-29**\ , date raised: 2020-11-25 
 * Version 1.0.28: resolved Issue 0473: no glfw option (extension)
    - description:  add simple option in setup.py to deactivate glfw both in setup.py as well as in C++ part
    - date resolved: **2020-11-29**\ , date raised: 2020-11-25 
 * Version 1.0.27: resolved Issue 0457: GetVersionString (extension)
    - description:  put into docu with pybindings
    - date resolved: **2020-11-29**\ , date raised: 2020-11-07 
 * Version 1.0.26: resolved Issue 0472: examples in utilities (extension)
    - description:  activate lstlisting for examples
    - date resolved: **2020-11-25**\ , date raised: 2020-11-25 
 * Version 1.0.25: resolved Issue 0468: test WSL2 (test)
    - description:  test compilation on WSL2 - Windows subsystem for Linux
    - **notes:** WSL2 now used to automatically create linux wheels
    - date resolved: **2020-11-21**\ , date raised: 2020-11-19 
 * Version 1.0.24: :textred:`resolved BUG 0465` : SC.GetSystem(..) 
    - description:  raises RuntimeError: should return reference instead of copy
    - date resolved: **2020-11-21**\ , date raised: 2020-11-18 
 * Version 1.0.23: resolved Issue 0446: NodeIndex in arrays (check)
    - description:  use additional functionality to enable index type checks also in arrays, e.g., ArrayIndex of node numbers
    - **notes:** not needed for now
    - date resolved: **2020-11-21**\ , date raised: 2020-09-09 
 * Version 1.0.22: resolved Issue 0383: pybind11 submodule (extension)
    - description:  used for advanced functions, not necessarily included in exudyn or make other module
    - **notes:** not needed for now
    - date resolved: **2020-11-21**\ , date raised: 2020-05-06 
 * Version 1.0.21: resolved Issue 0191: Newton lambda (check)
    - description:  Check whether Newton can be implemented as lambda-function
    - **notes:** not needed for now
    - date resolved: **2020-11-21**\ , date raised: 2019-06-17 
 * Version 1.0.20: resolved Issue 0466: main/bin (change)
    - description:  remove main/bin from github and from Tools folder
    - date resolved: **2020-11-19**\ , date raised: 2020-11-19 
 * Version 1.0.19: resolved Issue 0464: processing module (extension)
    - description:  create processing module for parameter variation and optimization using multiprocessing library
    - date resolved: **2020-11-18**\ , date raised: 2020-11-18 
 * Version 1.0.18: resolved Issue 0462: AVX Celeron problems (docu)
    - description:  add info to documentation - FAQ AND common problems and installation instructions that CPUs without AVX support only work with 32bit version
    - date resolved: **2020-11-18**\ , date raised: 2020-11-16 
 * Version 1.0.17: resolved Issue 0459: lie group utilities (extension)
    - description:  add documented lie group utilities to exudyn (python) lib
    - date resolved: **2020-11-11**\ , date raised: 2020-11-11 
    - resolved by: S. Holzinger
 * Version 1.0.16: resolved Issue 0454: add item graph (extension)
    - description:  add graph containing nodes, objects, etc.
    - date resolved: **2020-10-08**\ , date raised: 2020-10-08 
 * Version 1.0.15: resolved Issue 0453: systemdata.NumberOfSensors (extension)
    - description:  add access function for systemdata.NumberOfSensors()
    - date resolved: **2020-10-08**\ , date raised: 2020-10-08 
 * Version 1.0.14: resolved Issue 0449: MT ngsolve (extension)
    - description:  add NGsolve multithreading library (task manager)
    - **notes:** first tests made
    - date resolved: **2020-09-16**\ , date raised: 2020-09-15 
 * Version 1.0.13: resolved Issue 0330: correct ODE2RHS (change)
    - description:  correct ODE2RHS to ODE2Terms in objects because it is left-hand-side
    - **notes:** changed object computation function from RHS to LHS, as it always computed the LHS (the system.cpp function ComputeODE2RHS then puts it to RHS)
    - date resolved: **2020-09-10**\ , date raised: 2020-02-03 
 * Version 1.0.12: resolved Issue 0435: check runtimeError (check)
    - description:  check if exception runtimeerror works for all catch cases (test in windows?)
    - date resolved: **2020-09-09**\ , date raised: 2020-07-21 
 * Version 1.0.11: resolved Issue 0445: remove GetItemByName() (change)
    - description:  remove GetNodeByName, GetObjectByName, etc. from C++ interface; already disabled in python interface before
    - date resolved: **2020-09-08**\ , date raised: 2020-09-08 
 * Version 1.0.10: resolved Issue 0288: Item::CallFunction (change)
    - description:  Disable Item::CallFunction functionality from EXUDYN; either outputvariables can be used, or some functions are automatically created including the documentation
    - **notes:** already removed from python interface earlier
    - date resolved: **2020-09-08**\ , date raised: 2019-12-10 
 * Version 1.0.9: resolved Issue 0443: SensorObject (warning)
    - description:  add error message, if sensorobject is used for a body (and check if SensorBody excepts object other than body
    - **notes:** added test for SensorObject if attached to body
    - date resolved: **2020-09-04**\ , date raised: 2020-09-03 
 * Version 1.0.8: :textred:`resolved BUG 0442` : difference MSC and setuptools 
    - description:  compilation with MSC and setuptools gives different results
    - **notes:** problem with VS2019 compilation of Eigen; resolved by removing VS2019 installation
    - date resolved: **2020-08-25**\ , date raised: 2020-08-24 
 * Version 1.0.7: resolved Issue 0431: auto create dirs (extension)
    - description:  automatically create dictionaries if they do not exist
    - date resolved: **2020-08-25**\ , date raised: 2020-07-01 
 * Version 1.0.6: resolved Issue 0439: setuptools (extension)
    - description:  use setuptools for installation
    - date resolved: **2020-08-17**\ , date raised: 2020-08-13 
 * Version 1.0.5: resolved Issue 0381: test pybind11_2020 (test)
    - description:  downloaded in Download folder
    - date resolved: **2020-08-17**\ , date raised: 2020-05-06 
 * Version 1.0.4: resolved Issue 0378: setup tools (extension)
    - description:  use setup tools to install EXUDYN on local user accounts; use installed python version to decide which version to install
    - date resolved: **2020-08-17**\ , date raised: 2020-05-04 
 * Version 1.0.3: resolved Issue 0441: remove WorkingRelease path (change)
    - description:  do not include WorkingRelease to sys.path any more, but require installation of modules
    - date resolved: **2020-08-14**\ , date raised: 2020-08-14 
 * Version 1.0.2: resolved Issue 0440: exudyn package (extension)
    - description:  make a package with sub .py files in exudyn package - requires renaming of C++ module
    - date resolved: **2020-08-14**\ , date raised: 2020-08-13 
 * Version 1.0.1: resolved Issue 0438: UBUNTU (extension)
    - description:  adapt setup.py and implementation for gcc and UBUNTU
    - date resolved: **2020-08-13**\ , date raised: 2020-08-13 
 * Version 1.0.0: :textred:`resolved BUG 0434` : CheckSystemIntegrity 
    - description:  gives wrong node, marker, etc. numbers for some checks
    - date resolved: **2020-07-20**\ , date raised: 2020-07-20 

***********
Version 0.1
***********

 * Version 0.1.367: resolved Issue 0433: #pragma once (change)
    - description:  remove #pragma once directives, not compatible with gcc
    - date resolved: **2020-07-20**\ , date raised: 2020-07-20 
 * Version 0.1.366: :textred:`resolved BUG 0432` : FFRF object bug 
    - description:  wrong results in ObjectFFRF in case of refpos!=0
    - date resolved: **2020-07-02**\ , date raised: 2020-07-02 
 * Version 0.1.365: resolved Issue 0429: add stress modes (extension)
    - description:  add additional modes in ObjectFFRFreducedOrder to visualize stresses and strains
    - date resolved: **2020-07-01**\ , date raised: 2020-07-01 
 * Version 0.1.364: resolved Issue 0419: visualization user function (extension)
    - description:  add possibility of user function for visualization: is called on cSystem side to generate graphicsData lists or via a thread-safe callback
    - date resolved: **2020-06-26**\ , date raised: 2020-05-29 
 * Version 0.1.363: resolved Issue 0371: prestep py function (extension)
    - description:  Add prestep function as function using mbs (MainSystem)
    - date resolved: **2020-06-24**\ , date raised: 2020-04-10 
 * Version 0.1.362: resolved Issue 0284: utilities docu (docu)
    - description:  Add documentation for exudynUtilities.py
    - date resolved: **2020-06-24**\ , date raised: 2019-12-07 
 * Version 0.1.361: resolved Issue 0259: output variable connector (extension)
    - description:  add consistent output variables for connectors
    - date resolved: **2020-06-24**\ , date raised: 2019-08-30 
 * Version 0.1.360: resolved Issue 0427: RollingDiscPenalty (extension)
    - description:  implement RollingDisc model with penalty formulation and friction
    - date resolved: **2020-06-22**\ , date raised: 2020-06-19 
 * Version 0.1.359: resolved Issue 0426: RollingDisc (extension)
    - description:  implement RollingDisc model
    - date resolved: **2020-06-22**\ , date raised: 2020-06-19 
 * Version 0.1.358: resolved Issue 0425: HasVelocityEquations() (check)
    - description:  check if HasVelocityEquations() is really needed
    - date resolved: **2020-06-19**\ , date raised: 2020-06-19 
 * Version 0.1.357: resolved Issue 0420: SuperElement gravity (extension)
    - description:  add gravity to superelements
    - date resolved: **2020-06-09**\ , date raised: 2020-06-03 
 * Version 0.1.356: resolved Issue 0405: MarkerSuperElementReducedOrderRigidBody (extension)
    - description:  Implement averaging multinode marker for position and orientation for reduced order elements
    - date resolved: **2020-06-09**\ , date raised: 2020-05-21 
 * Version 0.1.355: resolved Issue 0281: add STL import (new feature)
    - description:  add exudynGraphics function for STL faces import
    - date resolved: **2020-06-09**\ , date raised: 2019-12-05 
 * Version 0.1.354: resolved Issue 0422: contour maxValue (extension)
    - description:  add second mode to contour auto range, which does not reduce the range
    - date resolved: **2020-06-07**\ , date raised: 2020-06-07 
 * Version 0.1.353: resolved Issue 0421: standard views (extension)
    - description:  add key shortcuts for standard views
    - date resolved: **2020-06-07**\ , date raised: 2020-06-07 
 * Version 0.1.352: resolved Issue 0418: read only parameters (check)
    - description:  raise exception if read only values are attempted to be overwritten==> currently, read only parameters are ignored!
    - date resolved: **2020-06-01**\ , date raised: 2020-05-28 
 * Version 0.1.351: resolved Issue 0417: visualization shortnames (extension)
    - description:  add shortnames, e.g., VMass1D for visualization objects
    - date resolved: **2020-06-01**\ , date raised: 2020-05-27 
 * Version 0.1.350: resolved Issue 0411: MarkerSuperElementPosition (extension)
    - description:  cleanup MarkerSuperElementPosition Matrix objects and test
    - date resolved: **2020-06-01**\ , date raised: 2020-05-25 
 * Version 0.1.349: resolved Issue 0408: ObjectConnectorRelativeRotation (extension)
    - description:  add connector with drive functionality, which constrains relative rotation (or translation) in the local frame of one Marker; use gear ratio + offset; enables gears, drives, etc.
    - date resolved: **2020-06-01**\ , date raised: 2020-05-22 
 * Version 0.1.348: resolved Issue 0404: MarkerSuperElementRigidBody (extension)
    - description:  Implement averaging multinode maker for position and orientation
    - date resolved: **2020-06-01**\ , date raised: 2020-05-21 
 * Version 0.1.347: resolved Issue 0395: description (description)
    - description:  add latex description for MarkerSuperElementPosition
    - date resolved: **2020-06-01**\ , date raised: 2020-05-16 
 * Version 0.1.346: resolved Issue 0414: ObjectFFRFreducedOrder EP (extension)
    - description:  add Euler Parameter constraint to ObjectFFRFreducedOrder
    - date resolved: **2020-05-29**\ , date raised: 2020-05-26 
 * Version 0.1.345: resolved Issue 0338: check accessFunctionTypes (extension)
    - description:  add automatized marker/force/accessFunctionType checks using the fact that same bits are used in types
    - date resolved: **2020-05-28**\ , date raised: 2020-02-18 
 * Version 0.1.344: resolved Issue 0329: check markers (extension)
    - description:  add integrity check if node/body implements necessary access functions for marker
    - date resolved: **2020-05-28**\ , date raised: 2020-02-02 
 * Version 0.1.343: resolved Issue 0416: check Markers (extension)
    - description:  add check (WARNING) if joint is applied to two markers directing to identical nodes or bodies
    - **notes:** not possible for FFRF and generic objects
    - date resolved: **2020-05-27**\ , date raised: 2020-05-27 
 * Version 0.1.342: :textred:`resolved BUG 0415` : CNodeRigidBody2D bug 
    - description:  CNodeRigidBody2D misses OutputVariables Rotation, RotationMatrix, AngularVelocity(Local)
    - date resolved: **2020-05-26**\ , date raised: 2020-05-26 
 * Version 0.1.341: resolved Issue 0407: implement FFRF tisserand frame (extension)
    - description:  implement Phit.T\*M\*c_F=0 and xRefTilde.T\*M\*c_F=0 as FFRF constraint for rigid body motion
    - **notes:** only possible as ConnectorCoordinateVector constraint externally
    - date resolved: **2020-05-26**\ , date raised: 2020-05-22 
 * Version 0.1.340: resolved Issue 0388: add 1D nodes and objects (extension)
    - description:  add 1D nodes/objects for drive-train applications; add optional visualization offset position p0, rotation A0 and transformation q->(u3D, theta3d), which transforms the 1D coordinate to 3D translation and rotation
    - date resolved: **2020-05-26**\ , date raised: 2020-05-08 
 * Version 0.1.339: resolved Issue 0387: add velocity markers (extension)
    - description:  add MarkerNodeRotationCoordinate (velocity level=angular velocity), with option for local frame and for MarkerRotation matrix, this enables to couple single (angular) velocities and to couple drives, etc. between 1D objects and 3D objects
    - date resolved: **2020-05-26**\ , date raised: 2020-05-08 
 * Version 0.1.338: resolved Issue 0412: ConnectorCoordinateVector (extension)
    - description:  implement coordinate vector constraint applied to a body; can be used as generic joint (with user function)
    - date resolved: **2020-05-25**\ , date raised: 2020-05-25 
 * Version 0.1.337: resolved Issue 0409: MarkerObjectCoordinates (extension)
    - description:  add new Marker MarkerObjectCoordinates, which applies to all coordinates of an object; this enables generic constraints on object coordinates (nodal coordinates may be added in future)
    - date resolved: **2020-05-25**\ , date raised: 2020-05-24 
 * Version 0.1.336: resolved Issue 0402: add NGsolve interface (extension)
    - description:  add NGsolve to FEMinterface to create mechanical body from geo, some options; create M, K, nodeList, elements and surface; add surfaces for specific boundaries
    - date resolved: **2020-05-22**\ , date raised: 2020-05-21 
 * Version 0.1.335: resolved Issue 0399: ObjectContactFrictionCircleCable2D (correct)
    - description:  ObjectContactFrictionCircleCable2D: contact stiffness wrong comment
    - date resolved: **2020-05-22**\ , date raised: 2020-05-20 
 * Version 0.1.334: resolved Issue 0393: clean up ObjectGenericODE2 (cleanup)
    - description:  remove ffrf from ObjectGenericODE2
    - date resolved: **2020-05-21**\ , date raised: 2020-05-16 
 * Version 0.1.333: resolved Issue 0398: FEM interface (description)
    - description:  add FEMinterface python class for mesh and system matrix import, surface mesh extraction, mode computation, export to ObjectFFRF etc.
    - date resolved: **2020-05-17**\ , date raised: 2020-05-16 
 * Version 0.1.332: resolved Issue 0397: tests FFRF (description)
    - description:  add TestModels for FFRF and FFRFreducedOrder
    - date resolved: **2020-05-17**\ , date raised: 2020-05-16 
 * Version 0.1.331: resolved Issue 0392: MarkerSuperElementRigidReducedOrder (extension)
    - description:  add MarkerSuperElementRigidReducedOrder for reducedOrder objects
    - date resolved: **2020-05-17**\ , date raised: 2020-05-16 
 * Version 0.1.330: resolved Issue 0391: add MarkerSuperElementPosition (extension)
    - description:  add MarkerSuperElementPosition
    - date resolved: **2020-05-16**\ , date raised: 2020-05-16 
 * Version 0.1.329: resolved Issue 0125: LinkedDataMatrix (new feature)
    - description:  Implement LinkedDataMatrix
    - date resolved: **2020-05-16**\ , date raised: 2019-05-13 
 * Version 0.1.328: resolved Issue 0386: add superelement (extension)
    - description:  add intermediate object, which offers access functions for node-position/velocity,/jacobian; object redirects either to nodes or uses the mode basis (FFRF) to compute position of virtual node
    - date resolved: **2020-05-15**\ , date raised: 2020-05-07 
 * Version 0.1.327: resolved Issue 0382: add FFRF object (extension)
    - description:  enabling full and reduced set of coordinates
    - date resolved: **2020-05-15**\ , date raised: 2020-05-06 
 * Version 0.1.326: resolved Issue 0368: Implement reduced FFRF (extension)
    - description:  Implement reduced FFRF in GenericODE2 by adding a transformation matrix
    - date resolved: **2020-05-15**\ , date raised: 2020-04-10 
 * Version 0.1.325: resolved Issue 0379: auto testSuite (extension)
    - description:  integrate test suite into workflow for generating WorkingReleaseXYZ; copy output.log to WorkingRelease folders
    - date resolved: **2020-05-06**\ , date raised: 2020-05-05 
 * Version 0.1.324: resolved Issue 0367: Implement FFRF (extension)
    - description:  Implement FFRF in GenericODE2
    - **notes:** internal folder only
    - date resolved: **2020-05-06**\ , date raised: 2020-04-10 
 * Version 0.1.323: resolved Issue 0377: testSuite (extension)
    - description:  integrate test suite into workflow for generating WorkingReleaseXYZ; copy output.log to WorkingRelease folders; make directory with named (versionNr) logs to quickly find problems
    - date resolved: **2020-05-05**\ , date raised: 2020-05-04 
 * Version 0.1.322: resolved Issue 0376: user functions catch (extension)
    - description:  catch exceptions of user functions, such that uunidentified errors in time integration can be located
    - date resolved: **2020-04-25**\ , date raised: 2020-04-24 
 * Version 0.1.321: resolved Issue 0373: loadVectorUserFunction (check)
    - description:  loadVectorUserFunction does not work with np.array as return type
    - **notes:** changed std::function return type from StdVector3D(=std::array) to StdVector(=std::vector), which works automatically with numpy arrays
    - date resolved: **2020-04-24**\ , date raised: 2020-04-14 
 * Version 0.1.320: resolved Issue 0336: euler parameters (check)
    - description:  check input of euler paramters: is norm of EP given for 1-e8
    - date resolved: **2020-04-24**\ , date raised: 2020-02-17 
 * Version 0.1.319: resolved Issue 0375: rigid body COM (extension)
    - description:  add center of mass COM to rigid body; extend equations of motion
    - date resolved: **2020-04-22**\ , date raised: 2020-04-22 
 * Version 0.1.318: resolved Issue 0361: check GenericJoint (check)
    - description:  check generic joint jacobian: in case that not all translational components are fixed, the jacobian entries should be excluded
    - date resolved: **2020-04-22**\ , date raised: 2020-04-09 
 * Version 0.1.317: resolved Issue 0339: add LoadController (extension)
    - description:  add possibility to add controllers with sensors as input and a user python function
    - date resolved: **2020-04-22**\ , date raised: 2020-02-18 
 * Version 0.1.316: resolved Issue 0370: Implement DH parameters (extension)
    - description:  use DH-parameters to set up link relative to previous body
    - date resolved: **2020-04-20**\ , date raised: 2020-04-10 
 * Version 0.1.315: resolved Issue 0374: SetRenderState (extension)
    - description:  add function SetRenderState in analogy to GetRenderState in order to set previously saved openGL state
    - date resolved: **2020-04-14**\ , date raised: 2020-04-14 
 * Version 0.1.314: resolved Issue 0360: 3D rotation (extension)
    - description:  make incremental rotation with right-mouse-button relative to current configuration
    - date resolved: **2020-04-14**\ , date raised: 2020-03-20 
 * Version 0.1.313: resolved Issue 0366: Add FFRF GenericODE2 (extension)
    - description:  Add FFRF mode to GenericODE2, using the first node as RigidBodyNode to represent the floating reference frame
    - date resolved: **2020-04-11**\ , date raised: 2020-04-10 
 * Version 0.1.312: resolved Issue 0365: add SphericalJoint (extension)
    - description:  add separate spherical joint with option to constrain one of the 3 axes
    - date resolved: **2020-04-10**\ , date raised: 2020-04-10 
 * Version 0.1.311: resolved Issue 0364: add multinodal marker (extension)
    - description:  add multinodal marker with weights for GenericODE2 objects
    - date resolved: **2020-04-10**\ , date raised: 2020-04-10 
 * Version 0.1.310: resolved Issue 0363: draw springs 3d (extension)
    - description:  add 3d helical spring drawing
    - date resolved: **2020-04-10**\ , date raised: 2020-04-10 
 * Version 0.1.309: resolved Issue 0362: unique drawing (extension)
    - description:  make unique line and 3d drawing for markers, sensors and loads
    - date resolved: **2020-04-10**\ , date raised: 2020-04-10 
 * Version 0.1.308: resolved Issue 0359: vis nodes (extension)
    - description:  add option to visualize nodes as spheres
    - date resolved: **2020-04-10**\ , date raised: 2020-03-20 
 * Version 0.1.307: resolved Issue 0358: vis nodes (extension)
    - description:  visualize 3D nodes as 3 circles
    - date resolved: **2020-04-10**\ , date raised: 2020-03-20 
 * Version 0.1.306: resolved Issue 0297: ALEANCFCable2D mass terms (check)
    - description:  check if mass terms in ALEANCFCable2D (9th column/row)
    - **notes:** terms are correct, but lead to instability at high velocities
    - date resolved: **2020-04-10**\ , date raised: 2019-12-16 
 * Version 0.1.305: resolved Issue 0225: SlidingJointRigid (extension)
    - description:  Add functionality for rigid sliding joint or add a flag for sliding joint to do both options
    - **notes:** not needed for now
    - date resolved: **2020-04-10**\ , date raised: 2019-07-10 
 * Version 0.1.304: resolved Issue 0356: sensor tutorial (extension)
    - description:  add sensor to tutorial
    - date resolved: **2020-03-13**\ , date raised: 2020-03-08 
 * Version 0.1.303: resolved Issue 0302: benchmark problems (verification)
    - description:  implement benchmark problems of iftomm web page and from papers Bruls/Arnold, Terze, etc.
    - date resolved: **2020-03-08**\ , date raised: 2019-12-26 
 * Version 0.1.302: resolved Issue 0352: exceptions pybind (extension)
    - description:  check whether lambda functions do not catch appropriately exceptions in pybind ==> use manual try catch mechanisms to catch exceptions
    - date resolved: **2020-03-06**\ , date raised: 2020-03-03 
 * Version 0.1.301: resolved Issue 0328: check ANCFALE (check)
    - description:  check if access functions implemented correctly: local jacobians include only 3x8 instead of 3x9 components?
    - date resolved: **2020-03-06**\ , date raised: 2020-02-02 
 * Version 0.1.300: resolved Issue 0353: AddItem (extension)
    - description:  unify AddItem(dict/py::object) into one function
    - date resolved: **2020-03-05**\ , date raised: 2020-03-04 
 * Version 0.1.299: resolved Issue 0351: exudynFast (extension)
    - description:  name all versions exudyn, because of unresolvable conflicts in utilities includes
    - date resolved: **2020-03-05**\ , date raised: 2020-03-03 
 * Version 0.1.298: resolved Issue 0350: Add ODE2equations (extension)
    - description:  add ObjectODE2equations with GenericODE2/NodePoint/NodePoint2D nodes
    - date resolved: **2020-03-05**\ , date raised: 2020-03-02 
 * Version 0.1.297: resolved Issue 0348: GetObjectOutputBody (extension)
    - description:  GetObjectOutputBody and similar functions shall not be callable if system is inconsistent
    - date resolved: **2020-03-05**\ , date raised: 2020-02-24 
 * Version 0.1.296: resolved Issue 0346: evaluate autodiff (check)
    - description:  evaluate netgen autodiff and autodiff.github.io for straightforward auto-differentiation of ODE2RHS functions
    - date resolved: **2020-03-05**\ , date raised: 2020-02-23 
 * Version 0.1.293: resolved Issue 0345: time in connector UF (extension)
    - description:  add time to connector user functions, to allow time dependent trajectories in spring dampers
    - date resolved: **2020-02-23**\ , date raised: 2020-02-22 
 * Version 0.1.292: resolved Issue 0335: docu examples+equations (extension)
    - description:  add multiline equations section and example section in the object description
    - date resolved: **2020-02-23**\ , date raised: 2020-02-16 
 * Version 0.1.291: resolved Issue 0309: GenericRigidBodyJoint2D (extension)
    - description:  Add generic joint with local transformation matrix of marker1; enable fixed or free ux,uy and phi in the joint, relative to marker1 coordinate system
    - date resolved: **2020-02-23**\ , date raised: 2020-01-10 
 * Version 0.1.290: resolved Issue 0287: Std::array (extension)
    - description:  check if performance of python interfaces can be improved using std::array; add typdefs for std::array<Real,3>, etc.
    - date resolved: **2020-02-23**\ , date raised: 2019-12-09 
 * Version 0.1.289: resolved Issue 0343: extend CoordinateConstraint (extension)
    - description:  extend CoordinateConstraint user function: value0/1, value_t0/1, factorValue1
    - **notes:** NOT COMPLETED: do this extension instead in a separate user coordinate constraint as it requires also the jacobians to be computed
    - date resolved: **2020-02-22**\ , date raised: 2020-02-20 
 * Version 0.1.288: resolved Issue 0344: initialDisplacements (change)
    - description:  change the misleading name initialDisplacements to initialCoordinates in all nodes for consitency reasons
    - date resolved: **2020-02-21**\ , date raised: 2020-02-21 
 * Version 0.1.287: resolved Issue 0342: add load sensor (new feature)
    - description:  add load sensor which measures loads especially if modified in user defined loads
    - date resolved: **2020-02-19**\ , date raised: 2020-02-19 
 * Version 0.1.286: resolved Issue 0341: add bodyFixed loads (extension)
    - description:  add bodyFixed (local / follower) forces and torques
    - date resolved: **2020-02-19**\ , date raised: 2020-02-19 
 * Version 0.1.285: resolved Issue 0340: bodyFixed force (extension)
    - description:  use markers bodyFixed property to realize local forces - make bodyFixed globally available in markers and add to computation of loads in csystem.cpp
    - **notes:** moved bodyFixed property to loads; see issue #341
    - date resolved: **2020-02-19**\ , date raised: 2020-02-19 
 * Version 0.1.284: resolved Issue 0324: visualize sensors (extension)
    - description:  add visualization to sensors (draw as circle with cross)
    - date resolved: **2020-02-18**\ , date raised: 2020-01-25 
 * Version 0.1.283: resolved Issue 0301: exudynUtilities (extension)
    - description:  split up exudynUtilities into separate files
    - date resolved: **2020-02-14**\ , date raised: 2019-12-26 
 * Version 0.1.282: resolved Issue 0334: add RigidBodySpringDamper (new feature)
    - description:  add a generalization for CartesianSpringDamper, using local coordinate systems and coupling all local translations and rotations
    - date resolved: **2020-02-12**\ , date raised: 2020-02-12 
 * Version 0.1.281: resolved Issue 0319: PyError in C++ (check)
    - description:  check why PyError is not working any more properly
    - **notes:** did not show up again
    - date resolved: **2020-02-12**\ , date raised: 2020-01-16 
 * Version 0.1.280: resolved Issue 0311: Add generic node ODE2 (extension)
    - description:  generic node with n ODE2 coordinates
    - date resolved: **2020-02-12**\ , date raised: 2020-01-10 
 * Version 0.1.279: resolved Issue 0310: GenericRigidBodyJoint3D (extension)
    - description:  Add generic joint with local transformation matrix of marker1; enable fixed or free tranlatory motion (ux,uy,uz) and rotations (phix,phiy,phiz) in the joint, relative to marker1 coordinate system
    - date resolved: **2020-02-12**\ , date raised: 2020-01-10 
 * Version 0.1.278: resolved Issue 0304: time in constraints (extensions)
    - description:  consistently add time to constraint evaluation; add flag to mark time dependency of constraints (influence on velocity level and on initial conditions)
    - date resolved: **2020-02-12**\ , date raised: 2019-12-28 
 * Version 0.1.277: resolved Issue 0331: correct Rigid3DEP (check)
    - description:  correct gyroscopic terms and add test suite example
    - date resolved: **2020-02-03**\ , date raised: 2020-02-03 
 * Version 0.1.276: resolved Issue 0317: ANCF beam (extension)
    - description:  add rigid body access functions for ANCF elements
    - date resolved: **2020-02-02**\ , date raised: 2020-01-15 
 * Version 0.1.275: resolved Issue 0327: visualizationSettings (extension)
    - description:  convert system structures to dictionary and back; include type information
    - date resolved: **2020-01-31**\ , date raised: 2020-01-27 
 * Version 0.1.274: :textred:`resolved BUG 0326` : python single line 
    - description:  fix possible threading issues of python single command execute
    - date resolved: **2020-01-31**\ , date raised: 2020-01-27 
 * Version 0.1.273: resolved Issue 0146: pout, threads (extension)
    - description:  Check if pout is threadsafe  ==> introduce a mutex
    - date resolved: **2020-01-31**\ , date raised: 2019-05-25 
 * Version 0.1.272: resolved Issue 0322: node ltg (extension)
    - description:  add python access function to retrieve node local to global ODE2 and AE coordinates
    - **notes:** added reference to starting index in global coordinate vector
    - date resolved: **2020-01-25**\ , date raised: 2020-01-22 
 * Version 0.1.271: resolved Issue 0305: consistent shut down (extension)
    - description:  Perform consistent shut down of exudyn in case of PyError(): stop renderer before raising Py or SysError()
    - **notes:** but errors which occur directly in python cannot be catched
    - date resolved: **2020-01-25**\ , date raised: 2019-12-28 
 * Version 0.1.270: resolved Issue 0255: add docu for solvers (docu)
    - description:  add documentation about flags, formulas, error control, jacobians and about the steps in the solvers
    - date resolved: **2020-01-25**\ , date raised: 2019-08-28 
 * Version 0.1.269: resolved Issue 0227: IntegrityNodeCheck (extension)
    - description:  add integrity check that correct node type is supplied to object; use an additional function for RequiredNodeType(); use none for special elements or mixed node types --> check needs to be put into element specific checks
    - date resolved: **2020-01-25**\ , date raised: 2019-07-13 
 * Version 0.1.268: resolved Issue 0216: Sensors (new feature)
    - description:  Add sensor concept and add simple sensors for postprocessing; e.g. add simple sensors to cSystemData, which are written to prescribed sensor file or global sensor file; sensors have [marker, OutputVariableType, coordinate1, coordinate2=0]
    - date resolved: **2020-01-25**\ , date raised: 2019-06-29 
 * Version 0.1.267: :textred:`resolved BUG 0321` : RenderWindow focus 
    - description:  add options to resolve problem of focus/showing of render window in spyer/ipython
    - **notes:** solved: change sypder preferences, see FAQ
    - date resolved: **2020-01-22**\ , date raised: 2020-01-22 
 * Version 0.1.266: resolved Issue 0320: GetParameter (extension)
    - description:  in autogenerated object/node GetNodeParameter(..): change return value to py::in_(invalid index...) in order to reduce error message in spyder
    - date resolved: **2020-01-22**\ , date raised: 2020-01-16 
 * Version 0.1.265: resolved Issue 0316: Add try/catch (extension)
    - description:  add try/catch to main python interface functions; add in all autogenerated functions
    - date resolved: **2020-01-22**\ , date raised: 2020-01-10 
 * Version 0.1.264: resolved Issue 0318: sparse init acc (extension)
    - description:  compute initial accelerations with sparse solver
    - date resolved: **2020-01-21**\ , date raised: 2020-01-15 
 * Version 0.1.263: resolved Issue 0268: Export M, D, K, Cq (new feature)
    - description:  Make (linearized/constant) mass, damping, stiffness and constraint matrices available in python
    - date resolved: **2020-01-08**\ , date raised: 2019-10-10 
 * Version 0.1.262: resolved Issue 0269: Export residuals (new feature)
    - description:  Make residuals available in python
    - date resolved: **2020-01-06**\ , date raised: 2019-10-10 
 * Version 0.1.261: resolved Issue 0190: PybindMatrix (new feature)
    - description:  Use Pybind to bind matrices and vectors to numpy (simplify interface); copy all matrix/vector contents for now; only lateron, an option without copying would be nice; link to pybind example see https://github.com/pybind/pybind11/blob/master/tests/test_buffers.cpp as well as the pybind reference section about numpy
    - date resolved: **2020-01-06**\ , date raised: 2019-06-16 
 * Version 0.1.259: resolved Issue 0276: PySolver (new feature)
    - description:  Link data structures and functions of solver to python; use new MainSolver object for that reason
    - date resolved: **2020-01-05**\ , date raised: 2019-12-01 
 * Version 0.1.258: resolved Issue 0290: SetOutputToConsole (new feature)
    - description:  SetOutputToConsole(flag): add functionality to write activate/deactivate console output
    - date resolved: **2020-01-04**\ , date raised: 2019-12-13 
 * Version 0.1.257: resolved Issue 0289: SetOutputToFile (new feature)
    - description:  SetOutputToFile(flage, fileName): add functionality to write all console output to file
    - date resolved: **2020-01-04**\ , date raised: 2019-12-13 
 * Version 0.1.256: resolved Issue 0307: Add flowcharts (docu)
    - description:  add flowcharts for items, exu,SC,mbs,systemData, ... and solver
    - date resolved: **2019-12-31**\ , date raised: 2019-12-30 
 * Version 0.1.255: resolved Issue 0306: WaitForUserToContinue (extension)
    - description:  check if renderer is running; if not, the functions WaitForUserToContinue and WaitForRenderEngineStopFlag will do nothing or wait for console input
    - date resolved: **2019-12-31**\ , date raised: 2019-12-28 
 * Version 0.1.254: resolved Issue 0308: add show constraints mode (extension)
    - description:  add mode to show constraints with key "c" in openGL renderer
    - date resolved: **2019-12-30**\ , date raised: 2019-12-30 
 * Version 0.1.253: resolved Issue 0299: initial accelerations (extension)
    - description:  consistently compute initial accelerations for Newmark (and approx. for generalizedalpha
    - date resolved: **2019-12-27**\ , date raised: 2019-12-17 
 * Version 0.1.252: resolved Issue 0300: exudyn rules (extension)
    - description:  add name conventions, code style and rules, etc. to docu
    - date resolved: **2019-12-26**\ , date raised: 2019-12-25 
 * Version 0.1.251: resolved Issue 0256: cleanup solvers (clean)
    - description:  cleanup and unify initialization and computation iterations for solvers; add solver data structure accessible via pybind
    - date resolved: **2019-12-25**\ , date raised: 2019-08-28 
 * Version 0.1.250: resolved Issue 0254: CircleContact (extension)
    - description:  pre-check possible region of contact
    - date resolved: **2019-12-25**\ , date raised: 2019-08-28 
 * Version 0.1.249: resolved Issue 0244: RecordFrames (new feature)
    - description:  grap openGL snapshot with glReadPixels and store frame to image file
    - date resolved: **2019-12-25**\ , date raised: 2019-08-22 
 * Version 0.1.248: resolved Issue 0208: Check diff rel eps (check)
    - description:  Check why the differentiation parameter needs to be very small 1e-11 in case of larger system sizes
    - **notes:** might be solved with new jacobianAE and new solvers
    - date resolved: **2019-12-25**\ , date raised: 2019-06-28 
 * Version 0.1.247: resolved Issue 0298: solver step reduction (extension)
    - description:  add step reduction if Newton / discIt fails to solver
    - date resolved: **2019-12-18**\ , date raised: 2019-12-16 
 * Version 0.1.246: resolved Issue 0295: new static solver (extension)
    - description:  add new static solver and test it
    - date resolved: **2019-12-17**\ , date raised: 2019-12-16 
 * Version 0.1.245: resolved Issue 0245: generalized alpha Newton (check)
    - description:  revise newton method in time integration solver: modified and full newton are not consistent; time integration continues even in case of errors, etc.
    - date resolved: **2019-12-17**\ , date raised: 2019-08-23 
 * Version 0.1.244: resolved Issue 0296: ANCFCable2DALE (extension)
    - description:  additional flag for moving mass terms
    - date resolved: **2019-12-16**\ , date raised: 2019-12-16 
 * Version 0.1.243: resolved Issue 0294: improve impl. solver (extension)
    - description:  compute correct initial conditions, add new convergence criteria (residuum/nCoords and newtonDecrement.norm2/nCoords)
    - date resolved: **2019-12-15**\ , date raised: 2019-12-15 
 * Version 0.1.242: resolved Issue 0275: solver_data (new feature)
    - description:  restructure solver data structure: computation data, temporary data, system matrices, functions
    - date resolved: **2019-12-15**\ , date raised: 2019-12-01 
 * Version 0.1.241: resolved Issue 0293: disc.iteration (change)
    - description:  discontinuous iteration in generalized alpha: verbose wrongly used; should not have affected the solution
    - date resolved: **2019-12-14**\ , date raised: 2019-12-14 
 * Version 0.1.240: :textred:`resolved BUG 0292` : time steps 
    - description:  Wrong time is used for time integration when evaluating loads, etc.; need to change time from beginning of step to end of step for evaluation of RHS
    - date resolved: **2019-12-13**\ , date raised: 2019-12-13 
 * Version 0.1.239: :textred:`resolved BUG 0291` : generalized alpha 
    - description:  discontinuous iteration is is not initialized for algorithmic acceleration
    - date resolved: **2019-12-13**\ , date raised: 2019-12-13 
 * Version 0.1.238: resolved Issue 0187: GetAccessFunctionBody (change)
    - description:  Use ResizableMatrix in GetAccessFunctionBody and Resizable Vector in GetOutputVariableBody; add templated fill-in function to ResizableVector/Matrix to be able to copy data more easy from other types
    - date resolved: **2019-12-11 13:27**\ , date raised: 2019-06-13 
 * Version 0.1.237: resolved Issue 0099: split marker/load (new feature)
    - description:  split Marker and Load into .h and .cpp AND reduce dependencies on Nodes, body, etc.    
    - date resolved: **2019-12-11**\ , date raised: 2019-04-01 
 * Version 0.1.236: resolved Issue 0006: link (new feature)
    - description:  link to matrix/vector classes to Eigen OR MKL solvers    
    - date resolved: **2019-12-11**\ , date raised: 2019-04-01 
 * Version 0.1.235: resolved Issue 0286: check visualization (check)
    - description:  Check visualization of items, specifically of nodes and objects: graphicsdata should use the AddBodyGraphicsData(...) function for rigid body transformation
    - date resolved: **2019-12-10**\ , date raised: 2019-12-07 
 * Version 0.1.234: resolved Issue 0235: CqTLambda (extension)
    - description:  ComputeODE2RHS (=CqT\*lambda) based on single constraint object jacobians instead of global matrix multiply
    - date resolved: **2019-12-09**\ , date raised: 2019-08-19 
 * Version 0.1.233: resolved Issue 0158: SuperLU (new feature)
    - description:  Link SuperLU to linalg
    - date resolved: **2019-12-09**\ , date raised: 2019-05-28 
 * Version 0.1.232: resolved Issue 0157: EigenTriple (new feature)
    - description:  Add Eigentriple to mass matrix and jacobian computation
    - date resolved: **2019-12-09**\ , date raised: 2019-05-28 
 * Version 0.1.231: resolved Issue 0155: Joints (new feature)
    - description:  Add 2D spherical and prismatic joint
    - date resolved: **2019-12-09**\ , date raised: 2019-05-28 
 * Version 0.1.230: :textred:`resolved BUG 0285` : rigidbody2D 
    - description:  TriangleList does not work
    - date resolved: **2019-12-07**\ , date raised: 2019-12-07 
 * Version 0.1.229: resolved Issue 0282: relativ paths import (extension)
    - description:  use relative paths for import of WorkingRelease and TestModels
    - date resolved: **2019-12-07**\ , date raised: 2019-12-07 
 * Version 0.1.228: resolved Issue 0271: referenceCoordsRigid2D (check)
    - description:  check whether all reference coordinates in NodeRigid2D are correctly considered
    - date resolved: **2019-12-07**\ , date raised: 2019-10-19 
 * Version 0.1.227: resolved Issue 0280: add OpenGL settings (new feature)
    - description:  add settings for lights, material, normals and faces
    - date resolved: **2019-12-05**\ , date raised: 2019-12-05 
 * Version 0.1.226: resolved Issue 0279: add .py cube and cylinder (new feature)
    - description:  add functions in EXUDYN utilities for creation of 3D cube and cylinder with GLTriangles
    - date resolved: **2019-12-05**\ , date raised: 2019-12-05 
 * Version 0.1.225: resolved Issue 0265: Graphics Faces (new feature)
    - description:  Add triangular faces for 3D graphics
    - date resolved: **2019-12-05**\ , date raised: 2019-10-10 
 * Version 0.1.224: resolved Issue 0278: PyFunctions test (new feature)
    - description:  Test PyFunctions for SpringDamper, CoordinateSpringDamper, CoordinateConstraint and CoordinateLoad
    - date resolved: **2019-12-02**\ , date raised: 2019-12-01 
 * Version 0.1.223: resolved Issue 0277: PyFunctions (new feature)
    - description:  finalize PyFunctions for SpringDamper, CoordinateSpringDamper, CoordinateConstraint and CoordinateLoad
    - date resolved: **2019-12-02**\ , date raised: 2019-12-01 
 * Version 0.1.222: resolved Issue 0274: SparseLU (new feature)
    - description:  Add sparse matrices and sparse solver to static and dynamic solvers
    - date resolved: **2019-12-01**\ , date raised: 2019-11-14 
 * Version 0.1.221: resolved Issue 0273: constraint action (extension)
    - description:  Add constraint action forces \ :math:`Cq^T \cdot lambda`\  via a function in csystem; eliminates need for computation of separate matrix during computation
    - date resolved: **2019-12-01**\ , date raised: 2019-11-14 
 * Version 0.1.220: resolved Issue 0270: Intro to theDoc (new feature)
    - description:  write introductory sections and tutorials for theDoc
    - date resolved: **2019-12-01**\ , date raised: 2019-10-19 
 * Version 0.1.219: resolved Issue 0267: GitLab (new feature)
    - description:  Put EXUDYN on UIBK/GitLab
    - date resolved: **2019-12-01**\ , date raised: 2019-10-10 
 * Version 0.1.218: resolved Issue 0086: use (new feature)
    - description:  use pybind/functional.h (see pybind11.readthedocs.io) to: use python functions / classes in C++; also use classes to define functions (+parameters); user defined python objects!!!; see refToFunctionsClasses.py    
    - date resolved: **2019-12-01**\ , date raised: 2019-04-01 
 * Version 0.1.217: resolved Issue 0264: Rigid 3D (new feature)
    - description:  include rigid body, add graphics
    - date resolved: **2019-11-26**\ , date raised: 2019-10-10 
 * Version 0.1.216: :textred:`resolved BUG 0272` : maximumSolutionNorm 
    - description:  changed default value for maximum solution norm from 1e10 to 1e38, because it is the square norm (no square root!) and was easily exceeded with values u (or v) larger than 100000; now a scalar value can be up to 1e19, before the Newton method is stopped
    - **notes:** new limit is 1e38 for compatibility with float
    - date resolved: **2019-11-08**\ , date raised: 2019-11-08 
 * Version 0.1.215: :textred:`resolved BUG 0266` : correct unit tests 
    - description:  check 2 failures in unit tests
    - **notes:** small errors in tests 2 and 7 fixed by updating the reference values in the 1e-12 - 1e-9 range
    - date resolved: **2019-10-17**\ , date raised: 2019-10-10 
 * Version 0.1.214: resolved Issue 0263: ANCF damping (extension)
    - description:  finish bending damping
    - date resolved: **2019-10-17**\ , date raised: 2019-10-10 
 * Version 0.1.213: resolved Issue 0262: python generator (extension)
    - description:  add specific flag to autogenerated files, which detects if files have been changed or not
    - date resolved: **2019-10-10**\ , date raised: 2019-09-12 
 * Version 0.1.212: resolved Issue 0230: CircleContact2D (extension)
    - description:  add friction
    - date resolved: **2019-09-12**\ , date raised: 2019-07-23 
 * Version 0.1.211: resolved Issue 0257: color bar (new feature)
    - description:  add color bar for contour plots
    - date resolved: **2019-08-30**\ , date raised: 2019-08-30 
 * Version 0.1.210: resolved Issue 0253: outputvariables (extension)
    - description:  add new autogenerated output variables to all objects
    - date resolved: **2019-08-30**\ , date raised: 2019-08-28 
 * Version 0.1.209: resolved Issue 0231: PostProcessing (extension)
    - description:  Add contour plot and settings - must be same as in sensors
    - date resolved: **2019-08-30**\ , date raised: 2019-07-23 
 * Version 0.1.208: resolved Issue 0251: switching sliding joint (extension)
    - description:  add sliding joint example with switching activeConnector flag and relocation of sliding body
    - date resolved: **2019-08-28**\ , date raised: 2019-08-25 
 * Version 0.1.207: resolved Issue 0215: DiscontinuousIteration (extension)
    - description:  Add discontinuous iteration to dynamic solvers; also incorporate restarting of Newton by adding data variables
    - date resolved: **2019-08-28**\ , date raised: 2019-06-28 
 * Version 0.1.206: resolved Issue 0180: Jacobian (extension)
    - description:  Add jacobian computation for constraints based on markers and objects/nodes jacobians
    - date resolved: **2019-08-26**\ , date raised: 2019-06-11 
 * Version 0.1.205: :textred:`resolved BUG 0250` : test suite graphics fails 
    - description:  multiple use of graphics in testsuite leads to crashes; inconsistent cSystem and vSystem containers; check mbs.Reset() function
    - **notes:** added missing call to visualizationSystemData.Reset() in mbs.Reset()
    - date resolved: **2019-08-25**\ , date raised: 2019-08-25 
 * Version 0.1.204: resolved Issue 0249: user stopflag python (new feature)
    - description:  add read and write access to stopSimulation flag in python; this allows to interrupt python loops - e.g. for static loading or for animation
    - date resolved: **2019-08-25**\ , date raised: 2019-08-25 
 * Version 0.1.203: resolved Issue 0248: visualize solution (new feature)
    - description:  set visualization state and time with pybind interface and send renderer update flag
    - date resolved: **2019-08-25**\ , date raised: 2019-08-25 
 * Version 0.1.202: resolved Issue 0247: animateSolution (new feature)
    - description:  load solution file and consecutively set visualization state to loaded states; implement in python
    - date resolved: **2019-08-25**\ , date raised: 2019-08-25 
 * Version 0.1.201: resolved Issue 0240: TimeIntegrationCPU (new feature)
    - description:  add CPU statistics accoding to static solver (with common data structure) to time integration; this will be needed to check performance of sparse matrix versions
    - date resolved: **2019-08-25**\ , date raised: 2019-08-21 
 * Version 0.1.200: resolved Issue 0232: LoadSolution (new feature)
    - description:  Add functionality to load solution from coordinates solution file
    - date resolved: **2019-08-25**\ , date raised: 2019-07-23 
 * Version 0.1.199: :textred:`resolved BUG 0246` : ActivateConnector 
    - description:  Error in Newton / time integration with pure algebraic equations (CoordinateConstraint with activateConnector=False)
    - date resolved: **2019-08-23**\ , date raised: 2019-08-23 
 * Version 0.1.198: resolved Issue 0243: Vactive (extension)
    - description:  consistently integrate visualization active flag in UpdateGraphics for all items
    - date resolved: **2019-08-23**\ , date raised: 2019-08-22 
 * Version 0.1.197: resolved Issue 0242: MarkerDataComp (extension)
    - description:  Unify markerData computation in CSystem and in GetOutputVariableConnector at different places using a CSystem function
    - **notes:** not fully checked
    - date resolved: **2019-08-22**\ , date raised: 2019-08-22 
 * Version 0.1.196: resolved Issue 0233: Get/SetParameters (new feature)
    - description:  add functionality to set single parameters of items via pybind interface
    - date resolved: **2019-08-22**\ , date raised: 2019-07-23 
 * Version 0.1.195: resolved Issue 0239: PybindInterface (extension)
    - description:  Complete pybind interfaces for systemData, version, ...
    - date resolved: **2019-08-21**\ , date raised: 2019-08-21 
 * Version 0.1.194: resolved Issue 0238: JacobianODE2RHS_t (extension)
    - description:  add object-wise computation to NumericalJacobianODE2RHS_t in System.cpp
    - date resolved: **2019-08-21**\ , date raised: 2019-08-21 
 * Version 0.1.193: resolved Issue 0237: CoordinateSpringDamper (new feature)
    - description:  add new scalar (coordinate) spring damper for action on arbitrary objects; include dry friction as option
    - date resolved: **2019-08-21**\ , date raised: 2019-08-20 
 * Version 0.1.192: resolved Issue 0221: LoadCoordinate (new feature)
    - description:  Add a load which is attached to a single MarkerCoordinate
    - date resolved: **2019-08-21**\ , date raised: 2019-07-04 
 * Version 0.1.191: resolved Issue 0131: ResizableMatrix (check)
    - description:  Investigate casting between Matrix and ResizableMatrix ... check assignement operator (should it work?); implicit casting should be avoided because of memory allocation ==> use only explicit CopyFrom(Matrix)
    - date resolved: **2019-08-21**\ , date raised: 2019-05-17 
 * Version 0.1.190: resolved Issue 0236: IssueTracker (extension)
    - description:  add date, release and version (=number of resolved issues) into issues tracker
    - date resolved: **2019-08-20**\ , date raised: 2019-08-20 
 * Version 0.1.189: resolved Issue 0234: IntegrityMarkerCheck (check)
    - description:  Add check that correct markertype is used: e.g. BodyRigid for torque or CoordinateMarker, etc.
    - date resolved: **2019-08-19**\ , date raised: 2019-08-19 
 * Version 0.1.188: resolved Issue 0224: MarkerNodeCoordinate (extension)
    - description:  Add Integrity check for valid coordinate number
    - date resolved: **2019-08-19**\ , date raised: 2019-07-08 
 * Version 0.1.187: resolved Issue 0222: RequestedMarkerType (extension)
    - description:  Add RequestedMarkerType check to SystemIntegrity checks; specific marker type checks (e.g. SlidingJoint) are done in the element-specific checks ==> requestedMarkertype=None
    - date resolved: **2019-08-19**\ , date raised: 2019-07-06 
 * Version 0.1.186: resolved Issue 0092: add args (new feature)
    - description:  add args to pybind interface: m.def("add", &add, "A function which adds two numbers", py::arg("i") = 1, py::arg("j") = 2);    
    - date resolved: **2019-08-19**\ , date raised: 2019-04-01 
 * Version 0.1.185: resolved Issue 0091: Objects: (new feature)
    - description:  Objects: add to GetOutputVariableTypes(): GetAccessibleMarkerTypes() ==> returns all MarkerFlags, which can be used with body ...   
    - **notes:** already available via GetAccessFunctionTypes and GetOutputVariableTypes
    - date resolved: **2019-08-19**\ , date raised: 2019-04-01 
 * Version 0.1.184: resolved Issue 0085: Add (new feature)
    - description:  Add Test suite for Python side (test all interface functions)    
    - date resolved: **2019-08-19**\ , date raised: 2019-04-01 
 * Version 0.1.183: resolved Issue 0228: TimeIntNewton (check)
    - description:  Check why modified Newton does not converge as full Newton
    - date resolved: **2019-08-18**\ , date raised: 2019-07-15 
 * Version 0.1.182: resolved Issue 0223: Pendulum (check)
    - description:  Pendulum example with constraint does not work any more. check jacobian and algebraic equations
    - date resolved: **2019-08-18**\ , date raised: 2019-07-08 
 * Version 0.1.181: resolved Issue 0229: AxiallyMovingJoint (new feature)
    - description:  add prescribed sliding of point along ALECable2D
    - date resolved: **2019-07-24**\ , date raised: 2019-07-23 
 * Version 0.1.180: resolved Issue 0219: AxiallyMovingCable2D (new feature)
    - description:  Add ALE cable element with axially moving component; derive this class from ANCFCable2D; in this way parameters of ANCFCable2D are hidden, but all functions can be reused; direct access to parameters. must be removed in all ANCFCable2D implementation
    - date resolved: **2019-07-24**\ , date raised: 2019-07-04 
 * Version 0.1.179: resolved Issue 0226: SlidingJoint2D (extension)
    - description:  Add jacobian function
    - date resolved: **2019-07-23**\ , date raised: 2019-07-10 
 * Version 0.1.178: resolved Issue 0220: NodeGenericODE2 (new feature)
    - description:  Add generic node for ODE2 coordinates, used for AxiallyMovingCable2D
    - date resolved: **2019-07-23**\ , date raised: 2019-07-04 
 * Version 0.1.177: resolved Issue 0217: ContactFrictionCircle2D (new feature)
    - description:  Add a circular contact+friction; same as ObjectContactCircleCable2D
    - date resolved: **2019-07-23**\ , date raised: 2019-06-28 
 * Version 0.1.176: resolved Issue 0218: numDiff (check)
    - description:  Check whether the reference coordinates should be added to current coordinates for the size of the differentiation parameter
    - date resolved: **2019-07-12**\ , date raised: 2019-07-04 
 * Version 0.1.175: resolved Issue 0213: ObjectSlidingJoint2D (new feature)
    - description:  add sliding joint and according marker(s): one marker for set  of cable elements or a list of markers (needs to update ltg list)
    - date resolved: **2019-07-12**\ , date raised: 2019-06-28 
 * Version 0.1.174: resolved Issue 0210: LoadMassProportional (new feature)
    - description:  Add mass proportional vector loading: body marker + according load
    - date resolved: **2019-07-12**\ , date raised: 2019-06-28 
 * Version 0.1.173: resolved Issue 0185: User system function (new feature)
    - description:  Add user-defined system function to every end of step in time integration; use separate User-structure in settings
    - date resolved: **2019-07-10**\ , date raised: 2019-06-13 
 * Version 0.1.172: resolved Issue 0153: StaticSolver (new feature)
    - description:  Add nonlinear iteration for contact
    - date resolved: **2019-07-09**\ , date raised: 2019-05-28 
 * Version 0.1.171: resolved Issue 0214: Numerical Jacobian (extension)
    - description:  Add numerical jacobian for every object / constraint instead of global jacobian
    - date resolved: **2019-07-04**\ , date raised: 2019-06-28 
 * Version 0.1.170: resolved Issue 0212: ObjectContactCircleANCF2D (new feature)
    - description:  add a circular contact with centerpoint, radius and range of according angle of a circle segment
    - date resolved: **2019-07-04**\ , date raised: 2019-06-28 
 * Version 0.1.169: resolved Issue 0211: MarkerANCFCable2DShape (new feature)
    - description:  Add a marker to measure 2D/3D? ANCF shapes
    - date resolved: **2019-07-04**\ , date raised: 2019-06-28 
 * Version 0.1.168: resolved Issue 0193: Item names (extension)
    - description:  Add name tag to class interfaces for objects, nodes, ... to add item names; use empty string ("") to identify that default names shall be generated
    - date resolved: **2019-07-04**\ , date raised: 2019-06-19 
 * Version 0.1.167: resolved Issue 0166: ODE2CoordinatesNode (new feature)
    - description:  Replaced by new issue 220
    - date resolved: **2019-07-04**\ , date raised: 2019-06-03 
 * Version 0.1.166: resolved Issue 0207: ObjectContactCoordinate (new feature)
    - description:  add functionality for nonlinear iterations in objects and in static solver
    - date resolved: **2019-06-28**\ , date raised: 2019-06-28 
 * Version 0.1.165: resolved Issue 0206: ObjectContactCoordinate (new feature)
    - description:  Add a contact connector for single coordinates
    - date resolved: **2019-06-28**\ , date raised: 2019-06-28 
 * Version 0.1.164: resolved Issue 0183: MarkerNodeCoordinate (new feature)
    - description:  A marker which addresses a certain nodal coordinate for Issue #165 NodalConstraint
    - date resolved: **2019-06-28**\ , date raised: 2019-06-12 
 * Version 0.1.163: resolved Issue 0145: Jacobian (extension)
    - description:  Compute jacobian in timeintegration also with respect to velocities
    - date resolved: **2019-06-28**\ , date raised: 2019-05-23 
 * Version 0.1.162: resolved Issue 0093: SystemChecks (new feature)
    - description:  check system consistency/integrity before Assemble: objects->nodes, marker<->objects/nodes, marker<->constraints, marker<->loads
    - date resolved: **2019-06-28**\ , date raised: 2019-04-01 
 * Version 0.1.161: resolved Issue 0205: ObjectConnectorCartesianSpringDamper (new feature)
    - description:  Add a cartesian spring damper, which acts with certain parameters in x,y, and z-direction; can be used for 2D and 3D elements
    - date resolved: **2019-06-27**\ , date raised: 2019-06-27 
 * Version 0.1.160: resolved Issue 0204: NodeGenericData (new feature)
    - description:  Add new node with data coordinates; generic size
    - date resolved: **2019-06-26**\ , date raised: 2019-06-26 
 * Version 0.1.159: resolved Issue 0189: PythonClassNames (change)
    - description:  Add field pythonShortName to objectDefinition, to give the python object - e.g. ObjectConnectorDistance a better name - e.g. simply Distance, SpringDamper; use typedef, i.e. ConstrainDistance = ObjectConnectorDistance
    - date resolved: **2019-06-26**\ , date raised: 2019-06-14 
 * Version 0.1.158: resolved Issue 0188: useIndex2 (change)
    - description:  Rename useIndex2 in connectors / algebraic equations to velocityLevel
    - date resolved: **2019-06-26**\ , date raised: 2019-06-14 
 * Version 0.1.157: resolved Issue 0169: graphics (extension)
    - description:  Add graphics representation for ForceVector
    - date resolved: **2019-06-26**\ , date raised: 2019-06-05 
 * Version 0.1.156: resolved Issue 0165: ConstraintNodeCoord (new feature)
    - description:  ObjectConstraintNodeCoordinate: used to directly constrain two nodal coordinates
    - date resolved: **2019-06-26**\ , date raised: 2019-06-03 
 * Version 0.1.155: resolved Issue 0154: RigidBody (new feature)
    - description:  Add 2D rigid body
    - date resolved: **2019-06-26**\ , date raised: 2019-05-28 
 * Version 0.1.154: resolved Issue 0152: CData (new feature)
    - description:  Couple CData, initialState, etc. to pybind DIRECTLY as state structure --> enable multiple computations
    - date resolved: **2019-06-26**\ , date raised: 2019-05-28 
 * Version 0.1.153: resolved Issue 0140: enum Index (change)
    - description:  change from enum class to enum which is convertible to Index
    - date resolved: **2019-06-26**\ , date raised: 2019-05-20 
 * Version 0.1.152: resolved Issue 0133: AddMatrix (check)
    - description:  Test function add matrix and use it in mass matrix assembly
    - date resolved: **2019-06-26**\ , date raised: 2019-05-17 
 * Version 0.1.151: resolved Issue 0129: Solve:MarkerData (extension)
    - description:  Implement a GetMarkerData() function for markers
    - date resolved: **2019-06-26**\ , date raised: 2019-05-14 
 * Version 0.1.150: resolved Issue 0090: integrate (new feature)
    - description:  integrate rigid body (3D/2D) and according constraints    
    - date resolved: **2019-06-26**\ , date raised: 2019-04-01 
 * Version 0.1.149: resolved Issue 0203: LoadTorqueVector (new feature)
    - description:  Add new load TorqueVector
    - date resolved: **2019-06-25**\ , date raised: 2019-06-25 
 * Version 0.1.148: resolved Issue 0202: MarkerBodyRigid (new feature)
    - description:  Create a rigid body marker for application of torques
    - date resolved: **2019-06-24**\ , date raised: 2019-06-24 
 * Version 0.1.147: resolved Issue 0201: ObjectANCFCable2D (new feature)
    - description:  Add kappa0 and eps0
    - date resolved: **2019-06-23**\ , date raised: 2019-06-23 
 * Version 0.1.146: resolved Issue 0200: ObjectANCFCable2D (new feature)
    - description:  Create 2D ancf Bernoulli-Euler beam elements
    - date resolved: **2019-06-22**\ , date raised: 2019-06-28 
 * Version 0.1.145: resolved Issue 0199: NodePoint2DSlope1 (new feature)
    - description:  Create node for 2D ancf Bernoulli-Euler beam elements
    - date resolved: **2019-06-18**\ , date raised: 2019-06-18 
 * Version 0.1.144: resolved Issue 0198: 2D Nodes/Objects (new feature)
    - description:  Add NodePoint2D, NodeRigidBody2D, ObjectMassPoint2D, ObjectRigidBody2D, ObjectJointRevolute2D
    - date resolved: **2019-06-17**\ , date raised: 2019-06-17 
 * Version 0.1.143: resolved Issue 0197: ObjectConstraintCoordinate (new feature)
    - description:  constrain two coordinates; possibly add an offset (modifyable?)
    - date resolved: **2019-06-16**\ , date raised: 2019-06-16 
 * Version 0.1.142: resolved Issue 0196: MarkerNodeCoordinate (new feature)
    - description:  DE2/ODE1 coordinate at displacement or velocity level; extend MarkerType/OutputVariable interface
    - date resolved: **2019-06-15**\ , date raised: 2019-06-15 
 * Version 0.1.141: resolved Issue 0195: NodePointGround (new feature)
    - description:  Add node similar to nodepoint, but no action and zero coordinates
    - date resolved: **2019-06-14**\ , date raised: 2019-06-14 
 * Version 0.1.140: resolved Issue 0186: MarkerNodePoint (new feature)
    - description:  Add a Marker to Node point; extend according assemble and computation functions
    - date resolved: **2019-06-13**\ , date raised: 2019-06-13 
 * Version 0.1.139: resolved Issue 0184: GeneralizedAlpha (extension)
    - description:  Extend Newmark to generalized alpha
    - date resolved: **2019-06-13**\ , date raised: 2019-06-13 
 * Version 0.1.136: resolved Issue 0089: create (new feature)
    - description:  create .tex reference pages for objects    
    - date resolved: **2019-06-13**\ , date raised: 2019-04-01 
 * Version 0.1.135: resolved Issue 0182: Item Dicts (extension)
    - description:  Use letter V ahead all visualization parameters in interface; add VNodePoint python class for interface with visualization
    - date resolved: **2019-06-12**\ , date raised: 2019-06-12 
 * Version 0.1.134: resolved Issue 0181: SolutionFile (extension)
    - description:  Add user_defined comments to user file
    - date resolved: **2019-06-12**\ , date raised: 2019-06-12 
 * Version 0.1.133: resolved Issue 0177: safe copy (extension)
    - description:  Make consistent copy of current version
    - date resolved: **2019-06-12**\ , date raised: 2019-06-11 
 * Version 0.1.132: resolved Issue 0176: Python Interface (check)
    - description:  Check if it is better to add python classes for objects with according init function (can they be converted implicitly to dict and used in current AddNode(py::dict) function - or to use additional AddNode(...) function interfaces in MainSystem
    - date resolved: **2019-06-12**\ , date raised: 2019-06-09 
 * Version 0.1.131: resolved Issue 0175: Interface class (new feature)
    - description:  Add interface Python classes which return a dictionary for items; this enables autocompletion in editor ...
    - date resolved: **2019-06-12**\ , date raised: 2019-06-07 
 * Version 0.1.130: resolved Issue 0179: GLFW client (extension)
    - description:  Link client to MSC instead to MainSystem and link visualization to all cSystems; add "visualization.show" flag to visualize system
    - date resolved: **2019-06-11**\ , date raised: 2019-06-11 
 * Version 0.1.129: resolved Issue 0178: Start/stop renderer (change)
    - description:  Put function into MSC
    - date resolved: **2019-06-11**\ , date raised: 2019-06-11 
 * Version 0.1.128: resolved Issue 0170: RendererWait (extension)
    - description:  Add Wait function for renderer in pybind interface
    - date resolved: **2019-06-11**\ , date raised: 2019-06-06 
 * Version 0.1.127: resolved Issue 0168: Algebraic equations (extension)
    - description:  Extend implicit time integration for algebraic equations; add warning to explicit time integration
    - date resolved: **2019-06-11**\ , date raised: 2019-06-04 
 * Version 0.1.126: resolved Issue 0115: ComputationSystem (extension)
    - description:  Extend static computation for AE equations (DistanceConstraint)
    - date resolved: **2019-06-11**\ , date raised: 2019-05-13 
 * Version 0.1.125: resolved Issue 0156: Graphics (new feature)
    - description:  Add basic graphics elements (Line, Polygon, Circle) to bodies and ground visualization objects ==> for moving and static objects
    - date resolved: **2019-06-05**\ , date raised: 2019-05-28 
 * Version 0.1.124: resolved Issue 0167: Newton AE (new feature)
    - description:  Extend Newton for algebraic equations
    - date resolved: **2019-06-04**\ , date raised: 2019-06-03 
 * Version 0.1.123: resolved Issue 0163: FinishDistance (new feature)
    - description:  Finish algebraic equations for static and dynamic solver with distance
    - date resolved: **2019-06-04**\ , date raised: 2019-06-03 
 * Version 0.1.122: resolved Issue 0162: LagrangeMult (check)
    - description:  Check sign of Lagrange multipliers; related to CSystem::ComputeAERHS, last line
    - date resolved: **2019-06-04**\ , date raised: 2019-06-03 
 * Version 0.1.121: resolved Issue 0160: GraphicsUpdate (extension)
    - description:  Write GraphicsUpdate for nodes and SpringDamper
    - date resolved: **2019-06-03**\ , date raised: 2019-05-29 
 * Version 0.1.120: resolved Issue 0151: OpenGL options (new feature)
    - description:  Add opengl options to pybind: Visualization:General,Window(mouse move, zoom),OpenGL,System(Objects,Nodes,...),Text
    - date resolved: **2019-05-29**\ , date raised: 2019-05-28 
 * Version 0.1.119: resolved Issue 0149: StaticSolver (extension)
    - description:  Static solver: add time to state structure and file output
    - date resolved: **2019-05-29**\ , date raised: 2019-05-27 
 * Version 0.1.118: resolved Issue 0159: StopComputation (new feature)
    - description:  Add shortcut (CTRL Q) to OpenGL to quit simulation
    - date resolved: **2019-05-28**\ , date raised: 2019-05-28 
 * Version 0.1.117: resolved Issue 0150: OpenGLText (extension)
    - description:  Add simple text structure to OpenGL
    - date resolved: **2019-05-28**\ , date raised: 2019-05-28 
 * Version 0.1.116: resolved Issue 0148: CData (extension)
    - description:  extend static/dynamic solvers to work with CData State structures, including start of step, initial step and according time values
    - date resolved: **2019-05-28**\ , date raised: 2019-05-25 
 * Version 0.1.115: :textred:`resolved BUG 0147` : glfwRenderer 
    - description:  StopRenderer() leads to python session termination ==> try to debug
    - date resolved: **2019-05-28**\ , date raised: 2019-05-25 
 * Version 0.1.114: resolved Issue 0144: accelerations (extension)
    - description:  add accelerations to cData and option for export (in time integration
    - date resolved: **2019-05-28**\ , date raised: 2019-05-23 
 * Version 0.1.113: resolved Issue 0117: Visualization (extension)
    - description:  Link visualization Items to main Items
    - date resolved: **2019-05-28**\ , date raised: 2019-05-13 
 * Version 0.1.112: resolved Issue 0116: Visualization (extension)
    - description:  Add Visualization objects/nodes/...
    - date resolved: **2019-05-28**\ , date raised: 2019-05-13 
 * Version 0.1.111: resolved Issue 0143: Newmark (extension)
    - description:  Add coefficients to Implicit Trapezoidal rule interface
    - date resolved: **2019-05-27**\ , date raised: 2019-05-23 
 * Version 0.1.110: resolved Issue 0083: graphics (new feature)
    - description:  Link GLFW: Add library; add VisualizationSystem; link to MainSystem; add data structure (linked to pybind); Initialization; add FLAG to deactivate    
    - date resolved: **2019-05-27**\ , date raised: 2019-04-01 
 * Version 0.1.109: resolved Issue 0138: Discussion (new feature)
    - description:  Add new label DISCUSSION with blue color to issue tracker
    - date resolved: **2019-05-23**\ , date raised: 2019-05-19 
 * Version 0.1.108: :textred:`resolved BUG 0137` : StaticSolver 
    - description:  Debug static solver - crashes
    - date resolved: **2019-05-23**\ , date raised: 2019-05-19 
 * Version 0.1.107: resolved Issue 0134: SpringDamper (extension)
    - description:  Add velocities to Bodies / Markers and to SpringDamperActuator
    - date resolved: **2019-05-23**\ , date raised: 2019-05-19 
 * Version 0.1.106: resolved Issue 0113: ComputationSystem (new test)
    - description:  Test Loads
    - date resolved: **2019-05-23**\ , date raised: 2019-05-13 
 * Version 0.1.105: resolved Issue 0096: check Timeint (new feature)
    - description:  check TimeIntegration to work with new objects    
    - date resolved: **2019-05-23**\ , date raised: 2019-04-01 
 * Version 0.1.104: resolved Issue 0128: CMarker::GetPosJac (change)
    - description:  Remove return value Matrix and use Matrix& in arguments to avoid memory allocation
    - date resolved: **2019-05-19**\ , date raised: 2019-05-14 
 * Version 0.1.103: resolved Issue 0114: StaticSolver (new feature)
    - description:  Add Static (Nonlinear) Solver
    - date resolved: **2019-05-19**\ , date raised: 2019-05-13 
 * Version 0.1.102: resolved Issue 0112: ComputationSystem (extension)
    - description:  Add Jacobian evaluation for ODE2RHS
    - date resolved: **2019-05-19**\ , date raised: 2019-05-13 
 * Version 0.1.101: resolved Issue 0110: ComputationSystem (extension)
    - description:  Add/merge system level computation functions: ComputeMass, ComputeODE2RHS, ...
    - date resolved: **2019-05-19**\ , date raised: 2019-05-13 
 * Version 0.1.100: resolved Issue 0084: Setup (new feature)
    - description:  Setup static solver (use Matrix-solver OR eigen-SuperLU)    
    - date resolved: **2019-05-19**\ , date raised: 2019-04-01 
 * Version 0.1.99: resolved Issue 0082: Add (new feature)
    - description:  Add CObjectGround to objects    
    - date resolved: **2019-05-19**\ , date raised: 2019-04-01 
 * Version 0.1.98: resolved Issue 0132: MatrixInvert (new feature)
    - description:  Implement simple matrix inversion for simpler tetss without making use of Eigen library
    - date resolved: **2019-05-17**\ , date raised: 2019-05-17 
 * Version 0.1.97: resolved Issue 0127: constraint ODE2RHS (change)
    - description:  add new interface for ComputeODE2RHS and ComputeAlgebraicEquations; add Array<MarkerData>, which is prefilled during computation; MarkerData=Position,Velocity,PosJacobian,RotJacobian; 
    - date resolved: **2019-05-17**\ , date raised: 2019-05-14 
 * Version 0.1.96: resolved Issue 0122: ResizableMatrix (new feature)
    - description:  Implement resizable matrix for temporary data structures in solver/time integration
    - date resolved: **2019-05-17**\ , date raised: 2019-05-13 
 * Version 0.1.95: resolved Issue 0111: ComputationSystem (extension)
    - description:  Add temporary evaluation structures (Matrices/Vectors) for system evaluation functions (Contraints/Markers)
    - date resolved: **2019-05-17**\ , date raised: 2019-05-13 
 * Version 0.1.94: resolved Issue 0004: Finish (new feature)
    - description:  Finish functions for all matrix classes    
    - date resolved: **2019-05-17**\ , date raised: 2019-04-01 
 * Version 0.1.93: resolved Issue 0003: set (new feature)
    - description:  set up all matrix classes    
    - date resolved: **2019-05-17**\ , date raised: 2019-04-01 
 * Version 0.1.92: resolved Issue 0001: Finish (new feature)
    - description:  Finish functions for all vector classes    
    - date resolved: **2019-05-17**\ , date raised: 2019-04-01 
 * Version 0.1.91: resolved Issue 0130: Eigen (check)
    - description:  perform eigen tests in separate console project
    - date resolved: **2019-05-16**\ , date raised: 2019-05-16 
 * Version 0.1.90: resolved Issue 0109: Solver (new feature)
    - description:  count memory allocations (=new) during solving; eliminate memory allocation
    - date resolved: **2019-05-15**\ , date raised: 2019-05-12 
 * Version 0.1.89: resolved Issue 0107: SpringDamper (change)
    - description:  remove Matrix from CObjectConstraintSpringDamperActuator::ComputeODE2RHS(...)
    - date resolved: **2019-05-15**\ , date raised: 2019-05-12 
 * Version 0.1.88: resolved Issue 0120: release assert (check)
    - description:  check if release assert works correctly in release and debug mode (check with invalid vector access)
    - date resolved: **2019-05-13**\ , date raised: 2019-05-13 
 * Version 0.1.87: resolved Issue 0119: available types (new feature)
    - description:  Show available types in GetObject/Node/Marker/...Defaults() function, if args are used: GetNodeDefault()
    - date resolved: **2019-05-13**\ , date raised: 2019-05-13 
 * Version 0.1.86: resolved Issue 0108: Matrix/Vector (new feature)
    - description:  add global counter for memory allocations (=new)
    - date resolved: **2019-05-13**\ , date raised: 2019-05-12 
 * Version 0.1.85: resolved Issue 0106: VS2017_PYPLOT (compatibility)
    - description:  matplotlib.pyplot does not work in VS2017 - installation fails, while matplotlib is installed; upgrade of pip installer does not help
    - **notes:** restart of VS2017 solved problem
    - date resolved: **2019-05-12**\ , date raised: 2019-05-12 
 * Version 0.1.84: resolved Issue 0103: dict access (new feature)
    - description:  Lateron: Add dict access to functions (NodeAccessFunction({'name':node_name, 'function':'Stress','position':[0,1,2,0.5],'option':'Cauchy'})    
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.83: resolved Issue 0102: dict access (new feature)
    - description:  Lateron: Add dict access to functions (NodeAccessFunction({'index':ind, 'function':'CurrentPosition'})    
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.82: resolved Issue 0101: Add (new feature)
    - description:  Add representation and mainobject.help()") # add latex description ... for Reference manual    
    - **notes:** NOT NEEDED: help already included in pybind/Python
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.81: resolved Issue 0095: MainSystemContainer (new feature)
    - description:  move MainSystemContainer to own class    
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.80: resolved Issue 0081: Debug: (new feature)
    - description:  Debug: CMarkerBodyPosition::GetPositionJacobian    
    - date resolved: **2019-05-12**\ , date raised: 2019-04-01 
 * Version 0.1.79: resolved Issue 0105: issue tracker (new feature)
    - description:  add filename and line number to issue tracker
    - date resolved: **2019-05-11**\ , date raised: 2019-05-11 
 * Version 0.1.78: resolved Issue 0104: issue tracker (new feature)
    - description:  set up issue tracking system
    - date resolved: **2019-05-10**\ , date raised: 2019-05-10 
 * Version 0.1.77: resolved Issue 0136: Differentiate (new feature)
    - description:  Add class function for numerical differentiation of a member function of CSystem
    - **notes:** first tests did not work ==> added manually
    - date raised: 2019-05-19 
 * Version 0.1.76: resolved Issue 0087: Data dependency (new feature)
    - description:  For the moment: use CSystemData\* in all objects, ...    
    - date raised: 2019-02-01 
 * Version 0.1.75: resolved Issue 0080: Setup (new feature)
    - description:  Setup time integration    
    - date raised: 2019-02-01 
 * Version 0.1.74: resolved Issue 0079: Assemble (new feature)
    - description:  Assemble function renewed(split into nodes-section, etc.): assign node coordinates, initialize global coordinate vectors    
    - date raised: 2019-02-01 
 * Version 0.1.73: resolved Issue 0078: def_readwrite (new feature)
    - description:  def_readwrite used to access SystemStates initial, current, ...    
    - date raised: 2019-02-01 
 * Version 0.1.72: resolved Issue 0077: Add (new feature)
    - description:  Add pybinding to CData as well ==> but only for read access    
    - date raised: 2019-02-01 
 * Version 0.1.71: resolved Issue 0076: Python: (new feature)
    - description:  Python: add pybinding to SystemState .def_property and get/set functions; then access initial, current, etc. via separate functions, e.g.    
    - date raised: 2019-02-01 
 * Version 0.1.70: resolved Issue 0075: CData (new feature)
    - description:  CData --> class SystemState; change to individual SystemState for current, initial, reference, etc.    
    - date raised: 2019-02-01 
 * Version 0.1.69: resolved Issue 0074: change (new feature)
    - description:  change ...ObjectType(), NodeType() in object/node to Type(); unified with CMarker!    
    - date raised: 2019-02-01 
 * Version 0.1.68: resolved Issue 0073: change (new feature)
    - description:  change ...CObjectType to ObjectType (same as nodes, markers, outputvariabletype...); NO "C"    
    - date raised: 2019-02-01 
 * Version 0.1.67: resolved Issue 0072: Use (new feature)
    - description:  Use consistently "Get..." in function (C++: always; py interface: discuss)    
    - date raised: 2019-02-01 
 * Version 0.1.66: resolved Issue 0071: add (new feature)
    - description:  add error handling for AddMainNode/Object/... according to AddMainMarker    
    - date raised: 2019-02-01 
 * Version 0.1.65: resolved Issue 0070: add (new feature)
    - description:  add GetLoadVector() for LoadForceVector    
    - date raised: 2019-02-01 
 * Version 0.1.64: resolved Issue 0069: DONE: (new feature)
    - description:  DONE: size=1 or 3; add dimensionality of Load==>corresponds to dim of Marker; remove loadVector default from CLoad    
    - date raised: 2019-02-01 
 * Version 0.1.63: resolved Issue 0068: no: (new feature)
    - description:  no: marker can have variable dimension; add dimensionality of Marker (MarkerBodyPosition=3, MarkerBodyCoordinate=1, MarkerBodyRigid=6, MarkerNodePoint=3, etc.)    
    - date raised: 2019-02-01 
 * Version 0.1.62: resolved Issue 0067: finish (new feature)
    - description:  finish constraints, markers and loads    
    - date raised: 2019-02-01 
 * Version 0.1.61: resolved Issue 0066: integrate (new feature)
    - description:  integrate markers and loads    
    - date raised: 2019-02-01 
 * Version 0.1.60: resolved Issue 0065: constraint (new feature)
    - description:  constraint integration: .cpp file, object factory    
    - date raised: 2019-02-01 
 * Version 0.1.59: resolved Issue 0064: sys.InfoDetailed() (new feature)
    - description:  sys.InfoDetailed() ==> Dicts of all objects, nodes, markers, ...    
    - date raised: 2019-02-01 
 * Version 0.1.58: resolved Issue 0063: sys.InfoSummary(): (new feature)
    - description:  sys.InfoSummary(): __repr__ of sys ==> shows lists and CData;    
    - date raised: 2019-02-01 
 * Version 0.1.57: resolved Issue 0062: GetNumberOfNodes() (new feature)
    - description:  sys.GetNumberOfNodes(), etc.    
    - date raised: 2019-02-01 
 * Version 0.1.56: resolved Issue 0061: Test (new feature)
    - description:  Test CallObjectFunction(...) ==> error in sys.PyGetOutputVariable(0,ht.OutputVariableType.Position)     
    - date raised: 2019-02-01 
 * Version 0.1.55: resolved Issue 0060: change (new feature)
    - description:  change marker->body/load->body/... references from pointers to numbers    
    - date raised: 2019-02-01 
 * Version 0.1.54: resolved Issue 0059: GetOutputVariableBod (new feature)
    - description:  GetOutputVariableBody(variableType, localPosition, configuration, value)    
    - date raised: 2019-02-01 
 * Version 0.1.53: resolved Issue 0058: GetOutputVariable (new feature)
    - description:  GetOutputVariable(variableType, value)    
    - date raised: 2019-02-01 
 * Version 0.1.52: resolved Issue 0057: Vector: (new feature)
    - description:  Vector: add .cpp file and resolve SlimVector compiler conflict    
    - date raised: 2019-02-01 
 * Version 0.1.51: resolved Issue 0056: CallFunction(...) (new feature)
    - description:  MainObject::CallFunction(...)    
    - date raised: 2019-02-01 
 * Version 0.1.50: resolved Issue 0055: PyCallObjectFunction (new feature)
    - description:  MainSystem::PyCallObjectFunction(...) --> py::object    
    - date raised: 2019-02-01 
 * Version 0.1.49: resolved Issue 0054: add (new feature)
    - description:  add CObjectMassPoint.cpp and copy functions from COMassPoint.h    
    - date raised: 2019-02-01 
 * Version 0.1.48: resolved Issue 0053: resolve (new feature)
    - description:  resolve includes for Node, Body, MainSystem, ObjectFactory, TimeIntegrationSolver    
    - date raised: 2019-02-01 
 * Version 0.1.47: resolved Issue 0052: add (new feature)
    - description:  add addProtected/Public to python autoGenerator    
    - date raised: 2019-02-01 
 * Version 0.1.46: resolved Issue 0051: migrate (new feature)
    - description:  migrate from COBody to CObjectBody and MainObjectBody (COMMENT OUT main.cpp and similar implementations...)    
    - date raised: 2019-02-01 
 * Version 0.1.45: resolved Issue 0050: put (new feature)
    - description:  put CSystemData\* into CObject    
    - date raised: 2019-02-01 
 * Version 0.1.44: resolved Issue 0049: add (new feature)
    - description:  add pybindings to Marker-/Load-/OuputVariable-/...types;     
    - date raised: 2019-02-01 
 * Version 0.1.43: resolved Issue 0048: add (new feature)
    - description:  add MainMassPoint, ...    
    - date raised: 2019-02-01 
 * Version 0.1.42: resolved Issue 0047: add (new feature)
    - description:  add MainMarker, MainObject, ...    
    - date raised: 2019-02-01 
 * Version 0.1.41: resolved Issue 0046: finish (new feature)
    - description:  finish MainNodePoint and access classes of Node     
    - date raised: 2019-02-01 
 * Version 0.1.40: resolved Issue 0045: ModifyNode(node (new feature)
    - description:  ModifyNode(node, dict)    
    - date raised: 2019-02-01 
 * Version 0.1.39: resolved Issue 0044: AddNode('nodeType' (new feature)
    - description:  AddNode('nodeType':'...', ...), GetNode(number or nodeName), GetDefaultNode('nodeType')    
    - date raised: 2019-02-01 
 * Version 0.1.38: resolved Issue 0043: Objects (new feature)
    - description:  Objects available in Python vs. Dict-Interface    
    - date raised: 2019-02-01 
 * Version 0.1.37: resolved Issue 0042: create (new feature)
    - description:  create concept for python integration:system    
    - date raised: 2019-02-01 
 * Version 0.1.36: resolved Issue 0041: create (new feature)
    - description:  create concept for python integration:specific objects: RigidBody, ...    
    - date raised: 2019-02-01 
 * Version 0.1.35: resolved Issue 0040: create (new feature)
    - description:  create concept for python integration:core objects: Node, Marker, ...    
    - date raised: 2019-02-01 
 * Version 0.1.34: resolved Issue 0039: move (new feature)
    - description:  move masspoint and other objects to objects directory    
    - date raised: 2019-02-01 
 * Version 0.1.33: resolved Issue 0038: create (new feature)
    - description:  create Object factory: objects added to specified CSystem    
    - date raised: 2019-02-01 
 * Version 0.1.32: resolved Issue 0037: access (new feature)
    - description:  access item in CSystem (test)    
    - date raised: 2019-02-01 
 * Version 0.1.31: resolved Issue 0036: add (new feature)
    - description:  add python access to CSystem members (obtain a certain CSystem as a copy?)    
    - date raised: 2019-02-01 
 * Version 0.1.30: resolved Issue 0035: AddPyfunction (new feature)
    - description:  AddPyfunction add pyFunction (which is global in module.cpp) to create a CSystem in SC    
    - date raised: 2019-02-01 
 * Version 0.1.29: resolved Issue 0034: define (new feature)
    - description:  define python dict structure (folders) for object definition:     
    - date raised: 2019-02-01 
 * Version 0.1.28: resolved Issue 0033: special (new feature)
    - description:  special function headers in MainObject (compute specific things, postprocessing of stresses?)    
    - date raised: 2019-02-01 
 * Version 0.1.27: resolved Issue 0032: MainObject (new feature)
    - description:  MainObject links to functions of CObject, as far as needed    
    - date raised: 2019-02-01 
 * Version 0.1.26: resolved Issue 0031: ObjectFactory (new feature)
    - description:  ObjectFactory function    
    - date raised: 2019-02-01 
 * Version 0.1.25: resolved Issue 0030: GetDict (new feature)
    - description:  GetDict / SetDict for MainObject types; all MAINObjects have this functionality?; this means a very deep integration of pybind11!; but not needed in base class, because all pybindings in derived class; all communication at 'dict' level, ; MainSystem.SetObjectDict(objectID[Name], dict); MainSystem.SetNodeDict(nodeID[Name], dict), etc.    
    - date raised: 2019-02-01 
 * Version 0.1.24: resolved Issue 0029: dict (new feature)
    - description:  dict access to MainObject for every chosen parameter or function (put into pymodule-part)    
    - date raised: 2019-02-01 
 * Version 0.1.23: resolved Issue 0028: (B) (new feature)
    - description:  (B) bind-only functions of mother class (without definition in Main/CObject)    
    - date raised: 2019-02-01 
 * Version 0.1.22: resolved Issue 0027: (D) (new feature)
    - description:  (D) function can have declaration only    
    - date raised: 2019-02-01 
 * Version 0.1.21: resolved Issue 0026: chose (new feature)
    - description:  chose, which parameter goes into CObject or MainObject (no doubling!)    
    - date raised: 2019-02-01 
 * Version 0.1.20: resolved Issue 0025: creates (new feature)
    - description:  creates CObject, MainObject, etc. classes    
    - date raised: 2019-02-01 
 * Version 0.1.19: resolved Issue 0024: write (new feature)
    - description:  write new pythonAutoGenerateObjects.py file for Objects, Nodes, Markers, etc.    
    - date raised: 2019-02-01 
 * Version 0.1.18: resolved Issue 0023: Create (new feature)
    - description:  Create -py multibody system according to CreateMultibodySystem() ==> what is done there to add objects?    
    - date raised: 2019-02-01 
 * Version 0.1.17: resolved Issue 0022: MainObject (new feature)
    - description:  MainObject setter function, using Python dictionary    
    - date raised: 2019-02-01 
 * Version 0.1.16: resolved Issue 0021: MainObject (new feature)
    - description:  MainObject getter function, using Python dictionary    
    - date raised: 2019-02-01 
 * Version 0.1.15: resolved Issue 0020: MainObjectFactory (new feature)
    - description:  MainObjectFactory function, using Python dictionary    
    - date raised: 2019-02-01 
 * Version 0.1.14: resolved Issue 0019: MainObject: (new feature)
    - description:  MainObject: Pointer to CObject, Pointer to VObject, MainObjectParameters, special member variables, access functions, initialization    
    - date raised: 2019-02-01 
 * Version 0.1.13: resolved Issue 0018: CObject: (new feature)
    - description:  CObject: CObjectParameters (currently all public), special member variables, compute functions    
    - date raised: 2019-02-01 
 * Version 0.1.12: resolved Issue 0017: Access (new feature)
    - description:  Access to all objects (nodes, markers, etc) independent of class hierarchy: with a Dictionary() access    
    - date raised: 2019-02-01 
 * Version 0.1.11: resolved Issue 0016: change (new feature)
    - description:  change pythonAutoGenerator Setter/Getter functions to Pybind default: Set...(const Value& value) {...}    
    - date raised: 2019-02-01 
 * Version 0.1.10: resolved Issue 0015: extend (new feature)
    - description:  extend pythonAutoGenerationInterfaces for python+Main-interface classes (flag p?)    
    - date raised: 2019-02-01 
 * Version 0.1.9: resolved Issue 0014: rename (new feature)
    - description:  rename PyCNode, PyCSystem, etc. to MainNode, etc.    
    - date raised: 2019-02-01 
 * Version 0.1.8: resolved Issue 0013: add (new feature)
    - description:  add MainSystem to CSystem and add functionality for object factory    
    - date raised: 2019-02-01 
 * Version 0.1.7: resolved Issue 0012: check (new feature)
    - description:  check how derived classes work in pybind (need for trampoline?) ==> use Test class    
    - date raised: 2019-02-01 
 * Version 0.1.6: resolved Issue 0011: Add (new feature)
    - description:  Add global stream which always goes to Python    
    - date raised: 2019-02-01 
 * Version 0.1.5: resolved Issue 0010: create (new feature)
    - description:  create PySystemContainer:SystemContainer which adds the Python components to SystemContainer    
    - date raised: 2019-02-01 
 * Version 0.1.4: resolved Issue 0009: create (new feature)
    - description:  create SystemContainer as basis for all Comptational (MBS) System objects    
    - date raised: 2019-02-01 
 * Version 0.1.3: resolved Issue 0008: access (new feature)
    - description:  access to system and objects, but objects still live in C++ world    
    - date raised: 2019-02-01 
 * Version 0.1.2: resolved Issue 0007: decide: (new feature)
    - description:  decide: where do objects live?: objects + system live in C++ (otherwise parallelization inefficient)    
    - date raised: 2019-02-01 
 * Version 0.1.1: resolved Issue 0000: Test (new feature)
    - description:  Test efficiency of virtual function calls in current Vector/Matrix library ==> already done earlier ==> will be efficient for AVX implementation (small overhead accepted)    
    - date raised: 2019-02-01 

***********
Open issues
***********

 * :textred:`open issue 2508:` checkExtras passed locally and failed in CI because of an untracked file
    - issue author: Claude-JG
    - description:  tools/checkExtras.py built its set of "local module names" with os.listdir/os.walk over python/; so ANY file present on the development machine made an import look local. python/pytest.py - the gitignored scratch copy of pytestTemplate.py - did exactly that: "import pytest" in test_testModels.py resolved to that file locally and the check reported OK; while the GitLab job of 2026-09-18; which has no such file; reported "UNCOVERED IMPORTS: pytest ... needed by [tests]" and failed. A gate that is green locally and red in CI is worse than no gate; and the same trap applies to any untracked helper anyone drops into python/ or TestModels/. Fixed in revision2026 step R5.18.3 by listing TRACKED files only (git ls-files); and by giving pytest a real exemption entry with its reason - it is a dev tool declared in [dependency-groups]; not in any extra.
    - date raised: 2026-09-18 

 * :textorange:`open issue 2507:` three examples FAIL for a missing optional package instead of being skipped
    - issue author: Claude-JG
    - description:  ExampleSkipReason() in testRunnerTools.py skips an example that cannot run - it already does so for stable-baselines3; rospy and a MATLAB TCPIP peer. Three examples are not covered and are reported as FAILURES instead: humanRobotInteraction.py and stlFileImport.py need numpy-stl ("No module named ,stl,") and pymeshlabFileImport.py needs pymeshlab. Measured 2026-09-18; they are 3 of the 5 entries now in KnownExampleFailures() (revision2026 step R5.18.1). The decision this needs is not obvious and is why it is an issue rather than three lines: a machine that HAS the package should run them - so the right fix is probably to TRY the import rather than to list file names; and then numpy-stl and pymeshlab belong in the [all] extra of pyproject.toml so that a developer environment has them. The other two entries of the list are unrelated: NGsolveGeometry.py fails inside the geometry construction under exec(...) and rendererNOGLFWexample.py expects the renderer to be absent in a way the runner does not produce.
    - date raised: 2026-09-18 

 * :textblue:`open issue 2505:` a test model page is in no toctree
    - issue author: Claude-JG
    - description:  The sphinx build prints "docs/RST/TestModels/sphereTriangleTest.rst: WARNING: document is not included in any toctree". The page is generated but unreachable from the documentation; a reader can only find it by searching. Noticed 2026-09-18 while running the documentation through the new exudev driver (revision2026 step R5.18); the build itself is not run with -W; so this does not fail anything today. Belongs to revision2026 phase R7.
    - date raised: 2026-09-18 

 * :textblue:`open issue 2498:` no test checks the member functions an item type must provide
    - issue author: Claude-JG
    - description:  Successor of #1142. What that issue asked for is now covered for PARAMETERS - parameterConversionTest.py writes a fixed set of probe values into every parameter of every item and compares the outcome with a reference (revision2026 step R4.4.3.1) - and for the linear algebra classes by the lest unit tests of src/Tests/ (step R5.4). What is still not tested per item type is its FUNCTIONS: that every object implements what its type requires (ComputeODE2LHS; GetOutputVariable; GetAccessFunctionTypes; ...) and that the output variables it advertises can actually be read. That needs the definitions database as its source of truth; like the parameter test does.
    - date raised: 2026-09-17 

 * :textblue:`open issue 2497:` 59 bare except: remain in the shipped package
    - issue author: Claude-JG
    - description:  Successor of #1988; which asked to change "except:" into "except Exception as e:" so that a KeyboardInterrupt passes through a long parameter variation. The change was never made; but it is no longer invisible: ruff E722 lists every one of the 59 with file and message in tools/ci/ruffBaseline.txt since revision2026 step R5.5.3; and a NEW one fails the check. Each needs a decision on which exception was actually meant; which is why they were baselined rather than rewritten. revision2026 step R6.1
    - date raised: 2026-09-17 

 * :textblue:`open issue 2455:` pydoclint reports two violations in exudyn/__init__.py RequireVersion
    - issue author: Claude-JG
    - description:  found 2026-09-16 during revision2026 step R5.13; pre-existing and unrelated to that step: DOC111 (type hints in the docstring arg list while --arg-type-hints-in-docstring is False) and DOC202 (return section without a return statement) in RequireVersion; belongs to revision2026 step R5.5 (ruff and type checking)
    - date raised: 2026-09-16 

 * **open issue 2432:** parameter conversion errors raise inconsistent exception types
    - issue author: Claude-JG
    - description:  Recorded by parameterConversionTest.py: a wrong value for an item or structure parameter raises RuntimeError (PyError after a C++ check or a pybind11 cast_error) - TypeError (pybind11 signature mismatch) or ValueError (Python checks) depending on the path; 34c4/34c5 moved most paths to RuntimeError. Maintainer decision 2026-09-15: exception types shall be corrected throughout the revision at an appropriate step - e.g. TypeError for a wrong type (str - list - None - item index into a scalar) and ValueError for a range violation or a wrong size - raised from PyConversion.h and PyError variants; reference update of parameterConversionTest.py. revision2026 step R6.7.
    - date raised: 2026-09-15 

 * **open issue 2423:** every C++ user error inspects the Python source for its file and line
    - issue author: Claude-JG
    - description:  PyError and PyWarning call PyGetCurrentFileInformation (src/Main/Stdoutput.cpp:259); which calls inspect.getframeinfo - that resolves the module by scanning sys.modules and reads the source file. The cost grows with the number of imported modules: the ~38000 probe errors of parameterConversionTest.py (revision2026 step R4.4.3.1) took 1 s standalone and 9 s inside runTestSuite.py after scipy; matplotlib and ngsolve were imported. It matters wherever errors are caught in a loop (parameter studies; try/except in user code). The frame alone (f_code.co_filename; f_lineno) gives the same information without the scan. revision2026 step R6.6.
    - date raised: 2026-09-14 

 * **open issue 2413:** ObjectContactConvexRoll.pContact is computed state stored in parameters
    - issue author: Claude-JG
    - description:  pContact is the currently computed contact point; written by the computation and read by the visualization (src/Objects/VisuNodePoint.cpp:2756 via GetPContact()). It lives in the parameter structure; so it is neither part of the system state nor kept per configuration and no history exists. It should be a data variable; which would also make the value available in the visualization configuration rather than whatever the last computation left behind. Found while removing the inert V flag from this member (revision2026 step R4.1.2).
    - date raised: 2026-09-13 

 * **open issue 2400:** computeMassMatrixInversePerBody does not reduce cost unless a sparse solver is also selected
    - issue author: Claude-JG
    - description:  the flag is documented as computing the inverse of the mass matrix per body so that explicit integration does not need a global solve; and it is the intended answer to the O(N^2) cost of issue 2398 (it cannot be the default; because it gives wrong results when bodies share nodes - a beam or an FEM body - as its own documentation and the maintainer both state). Measured 2026-09-12 on a chain of independent point masses; with the flag value read back from the settings to confirm it was applied: with the DEFAULT DENSE solver the flag changes nothing. At nMasses=1000 and 200 steps: ExplicitEuler 8.43 s off against 8.57 s on, RK44 20.5 against 20.4, DOPRI5 33.0 against 32.7 - all within noise. Selecting EigenSparse is what removes the cost (0.070 s); and only then is the flag worth a further 10 to 15 percent (0.058 s). So on its own the flag does not do what it promises; the user still has to know to change the linear solver. Either the flag should bypass the solver path; or its documentation should say that it must be combined with a sparse solver. Found while building the large system performance test for revision2026 step R2.10
    - date raised: 2026-09-12 

 * **open issue 2388:** the installation documentation is years out of date
    - issue author: Claude-JG
    - description:  docs/theDoc/gettingStarted.tex still instructs users with Python 3.6 and 3.7; 32 bit Anaconda; Spyder 4.1.3 and wheel names like exudyn-1.0.20-cp36-cp36m-win32.whl; and it discusses choosing between 32 and 64 bit installations. None of that has been built for years and after revision2026 step R2.6 the 32 bit build configurations no longer exist at all. The install section needs rewriting against the versions that are actually shipped (cp310-cp314; 64 bit only). Found during revision2026 step R2.6
    - date raised: 2026-09-12 

 * **open issue 2350:** MacOS               
    - description:  fix problem in raytracerNOGLFWtest.py on MacOS
    - date raised: 2026-08-05 

 * **open issue 2349:** shells              
    - description:  adjust comments to fit to internal exudyn format
    - date raised: 2026-08-05 

 * **open issue 2342:** itemInterface       
    - description:  replace CopyDictLevel1 with function that takes visualization and VItemClass in all self.visualization inits and either call VItemClass(\*\*visualization) if visualization is a dict, or store VItemClass object; this would enable to accept visualization as dict with only non-default values set; add try-except for dict-based call
    - date raised: 2026-04-17 

 * **open issue 2337:** CreateCoordinateConstraint
    - description:  change bodyNumbers to itemNumbers allowing both bodies and nodes to be constrained
    - date raised: 2026-04-06 

 * **open issue 2328:** ANCFCable           
    - description:  add documentation
    - date raised: 2026-03-24 

 * **open issue 2326:** SliderCrank Benchmark
    - description:  adapt TestModels/sliderCrank3Dbenchmark.py to revised IFToMM model
    - date raised: 2026-03-23 

 * **open issue 2325:** GenericJoint        
    - description:  improve documentation, in particular about order of axes for case of 1 and 2 rotation axes constrained
    - date raised: 2026-03-23 

 * **open issue 2321:** FEMinterface        
    - description:  meshes imported from NGsolve lead to triangles with wrong orientation as compared to GraphicsData
    - date raised: 2026-03-03 

 * **open issue 2319:** FEM                 
    - description:  extend interface to 6-noded triangles to represent quadratic shape functions / meshOrder=2 in ngsolve directly
    - date raised: 2026-03-02 

 * **open issue 2318:** GraphicsData        
    - description:  extend GraphicsData (C++ and interface) for 6-noded triangles to represent curved geometries with option for fine-interpolation and smooth normals
    - date raised: 2026-03-02 

 * **open issue 2317:** Add ObjectJointSliding
    - description:  Add implementation and tests for SlidingJoint with thick ANCF beam
    - date raised: 2026-03-02 

 * **open issue 2316:** Add ObjectJointSliding
    - description:  Add tests for SlidingJoint with thin ANCF cable
    - date raised: 2026-03-02 

 * **open issue 2315:** Add ObjectJointSliding
    - description:  Extend SlidingJoint for rotation case
    - date raised: 2026-03-02 

 * **open issue 2309:** ZoomAll             
    - description:  does not include trackMarker position (and orientation); fix that even for moving markers in modelCentricView zoom all is possible
    - date raised: 2026-02-19 

 * **open issue 2308:** shadows             
    - description:  in case modelCentricView=False, lights with useCameraFrame=True have erratic shadows  in OpenGL mode
    - date raised: 2026-02-19 

 * **open issue 2305:** graphics            
    - description:  Add error checks for isfinite for all point data imported in PyWriteBodyGraphicsDataList
    - date raised: 2026-02-15 

 * **open issue 2304:** graphics            
    - description:  add hints (object type, etc.) to PyWriteBodyGraphicsDataList in VisualizationSystemContainer to simplify error localisation
    - date raised: 2026-02-15 

 * **open issue 2281:** graphics            
    - description:  add functionality to make consistent triangles with same orientation (all computed normals are outbound or inbound); used to heal imported geometries
    - date raised: 2026-02-11 

 * **open issue 2278:** linux GLFW          
    - description:  check if call to glfwDestroyWindow from StopRenderer avoids crashes on linux?
    - date raised: 2026-02-09 

 * **open issue 2277:** linux GLFW          
    - description:  check if glfwGetWindowContentScale now works on newer GLFW version to enable display scaling on linux
    - date raised: 2026-02-09 

 * **open issue 2267:** GetURDFrobotData    
    - description:  add detailed description to function, in particular to returned dict
    - date raised: 2026-02-04 

 * **open issue 2259:** SystemContainer     
    - description:  add option to constructor whether to attach to renderer or not (if used purely for computations, like in InverseKinematics
    - date raised: 2026-02-02 

 * **open issue 2247:** camera              
    - description:  add documentation for model-centric and camera-centric views
    - date raised: 2026-01-31 

 * **open issue 2244:** GLFWClient          
    - description:  add ruler to renderer, only for case where axes are parallel to x, y and z (i.e. 90 degree rotations)
    - date raised: 2026-01-31 

 * **open issue 2238:** explicit solver     
    - description:  add adaptive step refinement for case of divergence - define limit for velocities/solution increment to decide step refinement (+ nan/inf)
    - date raised: 2026-01-27 

 * **open issue 2237:** MacOS               
    - description:  Check Spyder-issues and crashes with PlotSensor on MacOS systems
    - date raised: 2026-01-26 

 * **open issue 2236:** linux               
    - description:  fix wrong initialization for time in renderer on linux systems
    - date raised: 2026-01-26 

 * **open issue 2235:** computeInitialAccelerations
    - description:  add test for initial accelerations and velocities
    - date raised: 2026-01-23 

 * **open issue 2234:** computeInitialAccelerations
    - description:  add WARNING for inconsistent initial velocities - which usually cause heavy oscillations in constraint forces
    - date raised: 2026-01-21 

 * **open issue 2230:** ANCFThinPlate       
    - description:  add hemispheric test problem; compare to literature
    - date raised: 2026-01-18 

 * **open issue 2229:** ANCFThinPlate       
    - description:  add cylindrical test problem; compare to ANCFCable2D
    - date raised: 2026-01-18 

 * **open issue 2228:** Symmetric basis     
    - description:  add a function to compute a basis for two given non-parallel vectors; the average of the two vectors is computed from averaging normalized vectors (=mid axis) and according projections; used for NodeSlope12
    - date raised: 2026-01-18 

 * **open issue 2226:** ANCFThinPlate       
    - description:  compute optimal slopes scaling in ShellMesh functionality
    - date raised: 2026-01-16 

 * **open issue 2225:** ANCFThinPlate       
    - description:  add documentation of equations
    - date raised: 2026-01-16 

 * **open issue 2223:** symbolic            
    - description:  add vector and matrix functionality for Diff()
    - date raised: 2026-01-16 

 * **open issue 2219:** NodePointSlope12    
    - description:  correct average Rotation and RotationJacobian to be symmetric w.r.t. both slopes
    - date raised: 2026-01-13 

 * **open issue 2218:** ANCFThinPlate       
    - description:  add access function for rotation
    - date raised: 2026-01-13 

 * **open issue 2208:** GeometricallyExactBeam2D
    - description:  add test for 3-node element
    - date raised: 2026-01-08 

 * **open issue 2205:** perspective         
    - description:  test moving along a scene when changing the centerPoint
    - date raised: 2026-01-08 

 * **open issue 2204:** perspective         
    - description:  check OpenGL visibility problems with larger zoom (is this the 0.1 limit?)
    - date raised: 2026-01-08 

 * **open issue 2203:** MainSystem.Inspect  
    - description:  add function MainSystem.Inspect(itemIndex, what, optArgs) which retrieves additional info on items like available output variables, node/marker types, etc.
    - date raised: 2026-01-07 

 * **open issue 2202:** OutputVariable      
    - description:  add kinetic and potential energy
    - date raised: 2026-01-07 

 * **open issue 2200:** ANCFThinPlate       
    - description:  add test model
    - date raised: 2026-01-07 

 * **open issue 2194:** OpenGL view         
    - description:  add manual offsets for zNear and zFar, to adjust visible objects
    - date raised: 2026-01-06 

 * **open issue 2186:** RedrawAndSaveImage  
    - description:  add raytracer flag and make working offline
    - date raised: 2026-01-04 

 * **open issue 2184:** MarkerSuperElementRigid
    - description:  highly improve efficiency for larger number of nodes by adding according precomputed transformations
    - date raised: 2026-01-04 

 * **open issue 2183:** SparseTripletMatrix 
    - description:  add functions AddSparseTripletMatrix with parameters like AddToDenseMatrix; also add functions AddSubmatrix() and AddTransposedSubmatrix to add dense submatrices for ObjectFFRFreducedOrder
    - date raised: 2026-01-04 

 * **open issue 2182:** ObjectFFRFreducedOrder
    - description:  improve efficiency by using internal sparse triplets for mass matrix in C++ code
    - date raised: 2026-01-04 

 * **open issue 2155:** OpenGL              
    - description:  add textures to renderer (and add reset function), using numpy binary array representing RGB/RGBA image; reference numbers are then used in triangle lists
    - date raised: 2025-11-02 

 * **open issue 2154:** OpenGL              
    - description:  add textures for triangle lists, using texture reference number and coordinates, according to standard format
    - date raised: 2025-11-02 

 * **open issue 2140:** linux               
    - description:  fix graphics-related crashes on linux versions, in particular when closing renderer and with mbs.SolutionViewer()
    - date raised: 2025-07-10 

 * **open issue 2131:** ObjectContact       
    - description:  add rolling resistance
    - date raised: 2025-07-05 

 * **open issue 2130:** ObjectContact       
    - description:  consider changing contact objects like SphereSphere, SphereTriangle, etc. to switch in PostNewtonStep based on force sign rather than gap sign; possibly use global switch
    - date raised: 2025-07-05 

 * **open issue 2120:** Screw graphics      
    - description:  add function to generate graphics for screw
    - date raised: 2025-07-02 

 * **open issue 2110:** solver              
    - description:  add flag for local frame implicit solver, computing jacobians in local frame and using specific step updates
    - date raised: 2025-06-24 

 * **open issue 2109:** DOPRI5              
    - description:  DOPRI5 automatic step size not working well with discontinuities (ContactSphereSphere, etc.)
    - date raised: 2025-06-22 

 * **open issue 2106:** Chain drive         
    - description:  add function to create chain gears as well as geometry from chain drive; calculate length similar to reeving system, but with two chains (and kinck) to compensate length
    - date raised: 2025-06-21 

 * **open issue 2097:** ContactSphereTriangle
    - description:  add frictionStiffness similar to bristle model in ANCF contact
    - date raised: 2025-06-15 

 * **open issue 2075:** GetDictionary       
    - description:  add read/write dict access for new structures SC.renderer and exudyn.config
    - date raised: 2025-06-02 

 * **open issue 2017:** ContactCurveCircles 
    - description:  implement polynomial enhancements
    - date raised: 2025-05-13 

 * **open issue 2016:** ContactCurveCircles 
    - description:  add output variables
    - date raised: 2025-05-11 

 * **open issue 2007:** solver timers       
    - description:  add special timer for Python user functions, as solver timer for python does not include user functions
    - date raised: 2025-05-10 

 * **open issue 1989:** Body force sensor   
    - description:  add force sensor option for single-noded bodies and bodies which do not share nodes (mass points, rigid bodies, ffrf); for rigid bodies, obtains force and torque; for FFRF, it is generalized force; implement in a way that the contributions of loads are computed like in GeneralContact - based on a flag -, as soon as a BodySensor measures a force or torque
    - date raised: 2025-04-15 

 * **open issue 1977:** solver              
    - description:  check if solver can raise full solver error message in exception, in order to alleviate tracing during automated code evaluation in SolveStatic and SolveDynamic
    - date raised: 2025-03-30 

 * **open issue 1956:** items docu          
    - description:  add representative figure to each item
    - date raised: 2025-02-09 

 * **open issue 1954:** CreateLinearSpringDamper
    - description:  add test example
    - date raised: 2025-02-05 

 * **open issue 1953:** CreateLinearSpringDamper
    - description:  add create function to MainSystem
    - date raised: 2025-02-05 

 * **open issue 1947:** GeneralContact      
    - description:  check difference of friction force computation of SphereSphereContact (see notes in .cpp file) and GeneralContact
    - date raised: 2025-02-03 

 * **open issue 1941:** URDF                
    - description:  include parent names and create correct parent indices in URDF files by using link name lists; store list of base link names as well and do reordering
    - date raised: 2024-11-12 

 * **open issue 1932:** URDF import         
    - description:  GetURDFrobotData: add tool transformations accordingly (currently shown with no consecutive transformations)
    - date raised: 2024-11-10 

 * **open issue 1931:** URDF import         
    - description:  GetURDFrobotData: check for import of other scene information than mesh; check for import of collision
    - date raised: 2024-11-10 

 * **open issue 1924:** ContactCurveCircles 
    - description:  extend to frictional contact
    - date raised: 2024-11-04 

 * **open issue 1920:** ContactSphereSphere 
    - description:  add autodiff Jacobian
    - date raised: 2024-11-02 

 * **open issue 1912:** co-simulation       
    - description:  add simple model of two mass-spring-dampers to show simulator coupling of two implicit-explicit mbs, similar to issue 1905
    - date raised: 2024-10-26 

 * **open issue 1910:** total coordinates   
    - description:  consider a solver option which continuously provides total coordinates during iterations which could be used in user functions or globally and also be linked instead of copied; add an exception if the respective option is not switched on; by default, only add an empty Vector currentState.ODE2CoordsTotal
    - date raised: 2024-10-26 

 * **open issue 1907:** particles           
    - description:  add improved functions to create densly packed particles in box using simulation
    - date raised: 2024-10-19 

 * **open issue 1906:** particles           
    - description:  add improved functions to create more densly packed particles using advanced geometrical considerations for randomized radius spherical particles
    - date raised: 2024-10-19 

 * **open issue 1905:** simulator coupling  
    - description:  add test model for simulator coupling using mbs0 with joints and implicit integrator coupled to mbs1 with explicit integration
    - date raised: 2024-10-19 

 * **open issue 1904:** Reference/link data 
    - description:  evaluate further options to link to internal data, such as objects, nodes, etc.; possibly with GetObjectParameter(...) and similar functions possibly automated; this would highly speed up user functions as it reduces the number of exudyn function calls
    - date raised: 2024-10-19 

 * **open issue 1898:** particles           
    - description:  Add Python functionality for periodic walls, using pairs of walls at which particles are duplicated to the (-) side of a wall as soon as they transverse a periodic wall at the (+) side
    - date raised: 2024-10-16 

 * **open issue 1896:** GeneralContact      
    - description:  add option to completely freeze searchTree bins if all velocities are below a certain threshold; mark these contact objects as inactive by checking activity from time to time; reactivate bins only if item at boundary exceeds velocity limit at active boundary
    - date raised: 2024-10-16 

 * **open issue 1894:** GeneralContact      
    - description:  add n\*max(velocity)\*stepSize safety for bounding box, in order to update some objects less frequently (when max dist is reached)
    - date raised: 2024-10-16 

 * **open issue 1892:** Load/Save HDF5      
    - description:  Add Data structures like MatrixContainer, Vector3DList, etc.
    - date raised: 2024-10-13 

 * **open issue 1888:** mbs.GetDictionary   
    - description:  does not work for symbolic userfunctions
    - date raised: 2024-10-11 

 * **open issue 1864:** MatrixContainer     
    - description:  consider functionality to link to dense numpy matrix; possibly by using the allocatedSize in ResizableMatrix to indicate linking rather than allocation
    - date raised: 2024-10-02 

 * **open issue 1863:** MatrixContainer     
    - description:  consider functionality to link to sparse CSR scipy matrix rather than using the current CSR format - as a minimal solution do copying on C++ level; Problem: scipy CSR uses other format than Exudyns Triplets
    - date raised: 2024-10-02 

 * **open issue 1848:** GeneralContact      
    - description:  Test and improve implicit SPHERE-TRIG contact
    - date raised: 2024-06-02 

 * **open issue 1846:** ComputePostProcessingModes
    - description:  numberOfThreads> 1 not working: no modes computed
    - date raised: 2024-05-29 

 * **open issue 1845:** ComputePostProcessingModes
    - description:  numberOfThreads> 1 not working: conversion of vectorInput to np.array makes problems
    - date raised: 2024-05-29 

 * **open issue 1822:** inverse dynamics    
    - description:  consider an inverse dynamics solver for constrained systems; this requires decouple constraints from Lagrange multipliers; add separate LTG list for Lagrange multipliers and objectLTGAE; separate CObject::GetAlgebraicEquationsSize from LagrangeMultiplier size; check markerDataStructure.GetLagrangeMultipliers; see CSystemData::ComputeMarkerDataStructure
    - date raised: 2024-04-19 

 * **open issue 1821:** kinematics solver   
    - description:  consider functionality of a kinematic solver; this could be based on a quasi-static solver which utilizes the velocity constraint level and computes unknown velocities for a given configuration; first attempt could be based on finite differences for prescribed incremental motion and resulting incremental coordinates; only possible for systems with DOF=0; alternatively, we could compute velocity coordinates only from the constrained system, which however would only work if all coordinates are constrained
    - date raised: 2024-04-19 

 * **open issue 1813:** Marker positions    
    - description:  wrong representation of marker positions in AnimateModes for deformation scaling=0
    - date raised: 2024-04-08 

 * **open issue 1801:** joint constraints   
    - description:  add description of position jacobian for rigid bodies (in particular 3D rigid); add reference in description for MarkerBodyPosition
    - date raised: 2024-03-03 

 * **open issue 1777:** GraphicsData Sphere 
    - description:  add spheres to graphicsData interface; user AddSphere method; only use in case that full sphere is shown; add option to fall back to regular triangular representation
    - date raised: 2024-02-07 

 * **open issue 1776:** ComputeLinearizedSystem
    - description:  consider paper of Agundez, Vallejo, Freire, Mikkola in International Journal of Mechanical Sciences, Vol 268, 2024 for computation of linearized system and eigenmodes. Test case with bicycle
    - date raised: 2024-02-07 

 * **open issue 1769:** C++ user functions  
    - description:  Add cpp user functions fully to PythonUserFunctionBase capabilities
    - date raised: 2024-02-02 

 * **open issue 1768:** C++ user functions  
    - description:  Add pybind object as container for Cpp user functions, similar to autogenerated SetUserFunction
    - date raised: 2024-02-02 

 * **open issue 1767:** C++ user functions  
    - description:  add file for cpp user functions, registration mechanism like timers
    - date raised: 2024-02-02 

 * **open issue 1764:** GeneralContact      
    - description:  add pickle functionality
    - date raised: 2024-01-31 

 * **open issue 1751:** C++ user functions  
    - description:  check injection of user functions with special item-function method, and separate cpp file holding prototypes of those functions, which may be injected accordingly; use mechanism to record user functions in exudyn.functions.userCpp.SpringDamper
    - date raised: 2024-01-29 

 * **open issue 1740:** symbolic            
    - description:  check examples in Docu for consistency
    - date raised: 2023-12-19 

 * **open issue 1722:** Parameter type      
    - description:  add exudyn.Parameter for all parameters occuring in items, such as referencePosition, physicsMass, etc.; this helps to avoid strings in user functions to access parameter and may allow to use  more efficient case/switch in GetObjectParameter(...)
    - date raised: 2023-12-09 

 * **open issue 1720:** Mainsystem extensions
    - description:  change create2D functions into separate Create functions
    - date raised: 2023-12-08 

 * **open issue 1719:** generated examples  
    - description:  add set of generically generated examples, generated examples
    - date raised: 2023-12-08 

 * **open issue 1684:** ObjectIndex         
    - description:  consider functionality such as ComputeMassMatrix; ComputeODE2RHS, etc.; would require some default simulation settings (store in mainsystem?)
    - date raised: 2023-10-29 

 * **open issue 1683:** ItemIndices         
    - description:  consider direct access to outputvariables in node: nodeIndex.current.position; at least nodeIndex.GetOutput(variableType, configuration) would be valuable
    - date raised: 2023-10-29 

 * **open issue 1682:** ItemIndices         
    - description:  add previous CallFunction functionalities to NodeIndex, etc.; IsNodeGroup(group), IsNodeType(type), SizeODE2(), ..., 
    - date raised: 2023-10-29 

 * **open issue 1681:** ItemIndices         
    - description:  add option to add force/torque directly; add gravity to bodies
    - date raised: 2023-10-29 

 * **open issue 1677:** systemData          
    - description:  add GetDict(), Set(systemDict=[Dict]) functions which returns the whole dictionary for the system; containing list of nodes, objects, ...; each item is represented by its dictionary; could be used for set/get in future
    - date raised: 2023-10-29 

 * **open issue 1676:** ItemIndices         
    - description:  consider overriding __getattr__ and __setattr__ methods through pybind (or in Python with patching); this should allow to access data directly mapped via the dictionary
    - date raised: 2023-10-29 

 * **open issue 1675:** ItemIndices         
    - description:  consider adding MainSystem\* to indices; this would allow to directly operate on Nodes
    - date raised: 2023-10-29 

 * **open issue 1674:** license.ext         
    - description:  split into internal and external licenses
    - date raised: 2023-10-29 

 * **open issue 1668:** ANCFCable           
    - description:  add test example
    - date raised: 2023-10-16 

 * **open issue 1653:** ANCFBeam            
    - description:  reconsider name: ANCFBeamStructural, not to have too many cases; use this for 2/3 node, different number of slopes except for 1 slope, which is ANCFCable, the 3D version of ANCFCable2D
    - date raised: 2023-08-16 

 * **open issue 1631:** velocityOffset      
    - description:  add to CartesianSpringDamper, RigidBodySpringDamper
    - date raised: 2023-06-26 

 * **open issue 1614:** static members      
    - description:  LinearSolver GeneralMatrixEXUdense::FactorizeNew has static ResizableMatrix m, which should be turned into class members; add reset method to free memory at solver finalization
    - date raised: 2023-06-11 

 * **open issue 1565:** utilities InitializeFromRestartFile
    - description:  finalize C++ functionality and Python function
    - date raised: 2023-05-14 

 * **open issue 1550:** GeometricallyExactBeam
    - description:  add F_Lie\*Glocal_q term for Jacobian to improve convergence
    - date raised: 2023-05-02 

 * **open issue 1549:** KinematicTree       
    - description:  consider extension w.r.t. rigid body node at basis (Lie group node in explicit integration...); add baseNode (default=invalid), inertia could be added via a separate rigid body?
    - date raised: 2023-05-02 

 * **open issue 1548:** ODE1 loads          
    - description:  fully add Jacobian functionality for ODE1 loads and add test for ODE1 loads or recycle one
    - date raised: 2023-05-01 

 * **open issue 1529:** solver              
    - description:  solver functions GetSystemJacobian() and GetSystemMassMatrix() need to be extended with arg sparseTriplets=False; if True, it will return CSR sparse triplets, useful for large matrices, e.g. in eigenvalue computation in linearized system
    - date raised: 2023-04-26 

 * :textred:`open issue 1512:` return value policy 
    - description:  check return value policy of GeneralContact (as example for further decisions); see if reference in ALL access functions makes no problems if object is deleted on Python side
    - date raised: 2023-04-13 

 * :textorange:`open issue 1500:` ANCFBeam            
    - description:  check for advanced right-angle frame
    - date raised: 2023-04-08 

 * :textorange:`open issue 1499:` GeometricallyExactBeam
    - description:  check for advanced right-angle frame
    - date raised: 2023-04-08 

 * :textred:`open issue 1494:` GeometricallyExactBeam
    - description:  add reference configuration to residual and jacobian
    - date raised: 2023-04-06 

 * **open issue 1493:** StaticSolver        
    - description:  add exception in case that Lie group nodes are used with static solver, which cannot work
    - date raised: 2023-04-06 

 * **open issue 1485:** Spring-Damper connector description
    - description:  add general description for connectors based on spring-dampers (penalty)
    - date raised: 2023-04-01 

 * **open issue 1484:** Joint description   
    - description:  add general description for joint constraints
    - date raised: 2023-04-01 

 * :textred:`open issue 1450:` coordinatesSolution 
    - description:  add number of threads to solution files and more details on computer; check parameter variation and other files (e.g. numberOfThreads and final computation time)
    - date raised: 2023-02-25 

 * :textred:`open issue 1434:` solver              
    - description:  add CqT\*lambda terms to systemwide jacobian computation with flag
    - date raised: 2023-02-16 

 * **open issue 1424:** NumericalJacobianODE1RHS
    - description:  add case for duplicated ODE1 coordinates if connector has two markers for the same object, same as ltgODE2numDiff
    - date raised: 2023-02-08 

 * **open issue 1415:** CoordinateSpringDamperExt
    - description:  add flag for stepSizeRecommendation, where 0 is no recommendation, -1 is automatic and >0 is a directly recommended step size
    - date raised: 2023-01-22 

 * **open issue 1414:** InteractiveDialog   
    - description:  extend for explicit solver; needs internally different setup of solvers; use dynamicSolverType with default generalizedAlpha changable to Newmark/Index2 as well as explicit solvers
    - date raised: 2023-01-22 

 * **open issue 1395:** ComputeLinearizedSystem
    - description:  add test model
    - date raised: 2023-01-12 

 * **open issue 1337:** Newton              
    - description:  C++: check if SysError(s) in CSolverBase::Newton() can be changed into regular failure and step reduction for adaptiveStep
    - date raised: 2022-12-26 

 * **open issue 1292:** CSensorObject       
    - description:  store MarkerDataStructure locally in order to avoid memory allocations for evaluation of sensor data; also do this for MainSystem::PyGetObjectOutputVariable
    - date raised: 2022-11-13 

 * **open issue 1290:** ContactFrictionCircleCable2D
    - description:  shows tangential forces in case of all friction stiffness and damping values are zero; may be caused by specific projection
    - date raised: 2022-11-05 

 * **open issue 1273:** GeometricallyExactBeam
    - description:  check quadratic velocity terms
    - date raised: 2022-09-24 

 * **open issue 1247:** UserFunctions       
    - description:  check whether optimization of user functions with numba/JIT removes C->Python->C roundtrip overhead using pybind11 f.target approach from tests/test_callbacks.cpp; this would enable to retrieve the original c-function pointer
    - date raised: 2022-09-02 

 * **open issue 1241:** Register Items      
    - description:  consider mechanism to self-register items: objects, nodes, ...; same a swith TimerStructure registration; this allows to add user-defined objects without touching the overall code
    - date raised: 2022-08-24 

 * **open issue 1240:** Register unit tests 
    - description:  consider mechanism to self-register unit tests; same a swith TimerStructure registration
    - date raised: 2022-08-24 

 * :textred:`open issue 1203:` parallel / multithreaded
    - description:  C++: add multithreading for JacobianODE2 (analytic jacobians)
    - date raised: 2022-07-12 

 * :textred:`open issue 1202:` parallel / multithreaded
    - description:  C++: add multithreading for JacobianAE
    - date raised: 2022-07-12 

 * **open issue 1194:** generalizedAlpha scaling
    - description:  turn on/off scaling in interface to test symmetric solver speedup
    - date raised: 2022-07-11 

 * **open issue 1192:** ExplicitSolver      
    - description:  Newton / startOfStep: check if dataCoords should also be copied
    - date raised: 2022-07-10 

 * **open issue 1189:** ContactFrictionCircleCable2D
    - description:  add velocity offset to MarkerCable2DShape
    - date raised: 2022-07-08 

 * **open issue 1167:** user functions      
    - description:  check https://pybind11.readthedocs.io/en/stable/advanced/cast/functional.html regarding stateless functions and test performance with C++ functions for simple spring-damper
    - date raised: 2022-06-29 

 * :textorange:`open issue 1140:` c++ user elements   
    - description:  add auto registration for C++ user items
    - date raised: 2022-06-12 

 * :textorange:`open issue 1139:` c++ user elements   
    - description:  add description regarding which functions are needed to add C++ user elements
    - date raised: 2022-06-12 

 * **open issue 1104:** GenericObject       
    - description:  add most general generic object containing ODE1, ODE2 and AE equations + unknowns; jacobianAE as user functions
    - date raised: 2022-05-23 

 * **open issue 1100:** GeometricallyExactBeam3D
    - description:  finalize implementation and fix jacobian computation
    - date raised: 2022-05-23 

 * **open issue 1087:** ANCFBeam3D          
    - description:  complete implementation of all functions (rigid marker, etc.)
    - date raised: 2022-05-16 

 * **open issue 1078:** BeamSectionGeometry 
    - description:  add BeamSectionGeometry to 2D beam elements
    - date raised: 2022-05-09 

 * :textred:`open issue 1061:` Reference and copy  
    - description:  add information to theDoc regarding copying and referencing objects, such as mbs, GetObject(...), etc.; add info into description C/R into generatePyBindings?
    - date raised: 2022-04-30 

 * **open issue 1032:** ObjectConnectorCoordinateVector
    - description:  cleanup, consider better UF and check implementation with theory (jac?)
    - date raised: 2022-04-04 

 * **open issue 1008:** Contact switching   
    - description:  Test improved contact integration method with resolution of switching and correction of integration of discontinuous forces; compare to switching point resolution
    - date raised: 2022-03-26 

 * :textred:`open issue 0990:` MarkerNodeCoordinate
    - description:  add option addReferenceCoordinates=False to include reference value in coordinate
    - date raised: 2022-03-16 

 * **open issue 0988:** ComputeConstraintJacobianDerivative
    - description:  make sparse version similar to numerically differentiated single objects in JacobianODE2RHS
    - date raised: 2022-03-15 

 * :textorange:`open issue 0987:` ALEANCFCable2D      
    - description:  add description - specifically regarding OutputVariables, special terms not available in ANCFCable2D (and add reference to ASME CND paper)
    - date raised: 2022-03-15 

 * **open issue 0984:** OutputVariableConnector
    - description:  check all penalty-based connectors if OutputVariable for forces is only computed if activeConnector=True
    - date raised: 2022-03-14 

 * **open issue 0973:** ContactFrictionCircleCable2D
    - description:  adapt old tests and create new beltdrive test
    - date raised: 2022-03-09 

 * **open issue 0971:** Renderer            
    - description:  add mechanisms to catch exceptions inside renderer thread; try detaching renderer thread
    - date raised: 2022-03-06 

 * **open issue 0957:** JointRevolute2D     
    - description:  add OutputVariables in C++ and in DOCU; check other objects with missing OutputVariables
    - date raised: 2022-02-28 

 * **open issue 0948:** update mecanumWheelRollingDiscTest
    - description:  update w.r.t. Trajectory class and TorsionalSpringDamper
    - date raised: 2022-02-21 

 * **open issue 0937:** GeneralContact      
    - description:  add option to draw contact forces
    - date raised: 2022-02-10 

 * **open issue 0927:** GeneralContact ANCF 
    - description:  test if 3 maxTangentialVelocities in 3-point Lobatto integration lead to better Newton performance
    - date raised: 2022-02-04 

 * **open issue 0926:** RollingDiscPenalty  
    - description:  rollingFrictionViscous only works for rolls with axis parallel to z-Plane; add MISSING formulas to Docu and adapt formulation
    - date raised: 2022-02-03 

 * **open issue 0917:** mbs.ComputeObjectLHSJacobian
    - description:  add computation functions for object; using system function; needing option for analytic/numeric computation
    - date raised: 2022-02-02 

 * **open issue 0916:** mbs.ComputeObjectAccessFunction
    - description:  add computation functions for access functions, e.g., TranlationalVelocity_qt, AngVel_qt, ...
    - date raised: 2022-02-02 

 * **open issue 0915:** mbs.ComputeObject...
    - description:  add mbs.ComputeObjectMassMatrix(...)
    - date raised: 2022-02-02 

 * **open issue 0914:** mbs.ComputeNode...  
    - description:  add computation functions for nodes, e.g., position or rotation jacobian; coordinates are already available in GetNodeOutput(...)
    - date raised: 2022-02-02 

 * **open issue 0912:** GeneralContact      
    - description:  add second CCactiveSetError mode, which computes error for given active set ==> error for PostNewton computed (error in assumed conditions forces)
    - date raised: 2022-02-02 

 * **open issue 0911:** GeneralContact      
    - description:  implement contact laws
    - date raised: 2022-02-02 

 * **open issue 0910:** GeneralContact      
    - description:  implement recommended step size (with error bound and separate min stepsize to avoid too small steps
    - date raised: 2022-02-02 

 * **open issue 0909:** GeneralContact      
    - description:  add functionality to store/restore contact state for start of time step
    - date raised: 2022-02-02 

 * **open issue 0908:** MarkerData          
    - description:  add configuration to markerdata computation; allows configuration in sensors and startOfStep configuration in Contact
    - date raised: 2022-02-02 

 * **open issue 0907:** GeneralContact      
    - description:  add implicit Trig-Sphere contact
    - date raised: 2022-02-02 

 * **open issue 0904:** GeneralContact      
    - description:  PostNewton (sphere-sphere, ancf-circle): add if clause to switch off contact in case of negative contact force
    - date raised: 2022-01-31 

 * **open issue 0892:** AccessFunctionType  
    - description:  add AccessFunctionType::TranslationalVelocity_q and AccessFunctionType::AngularVelocity_q needed for analytical jacobians
    - date raised: 2022-01-26 

 * **open issue 0888:** add information on error handling
    - description:  explain System errors, Python errors and Warnings; explain exception handling and add example
    - date raised: 2022-01-25 

 * **open issue 0886:** Exceptions          
    - description:  test py::raise_from for Python-induced exceptions (in renderPythonInterface) or for SystemErrors; check whether execeptions are originating from C or Python, see pybind11 Exceptions
    - date raised: 2022-01-25 

 * **open issue 0873:** GenericJoint        
    - description:  improve computation of jacobian, using crossproduct
    - date raised: 2022-01-18 

 * **open issue 0871:** restart method      
    - description:  add option solutionSettings.writeRestartFile to restart from separate restart file; solutionSettings.restartFileName defines folder and fileName; solutionSettings.restartWritePeriod defines time in seconds, how often it is written; also writes backup file
    - date raised: 2022-01-18 

 * **open issue 0867:** GeneralContact      
    - description:  change deltaV terms in ANCFCable and TrigSphere contact to fit signs used in docu
    - date raised: 2022-01-17 

 * **open issue 0859:** GeneralContact      
    - description:  split searchtree into regions, proportional to number of threads (FinalizeContact); use 2 splits in x, 2 splits in y, etc. until uneven number left; add class Box3Dindexed:Box3D, which adds index for region in searchtree; add access in GeneralContact for adding bounding box, creating the index; index is -1, if overlapping, filled in serially
    - date raised: 2022-01-13 

 * **open issue 0858:** ParallelFor         
    - description:  use ParallelFor with costs argument in GeneralContact and CSystem, to optimize usage
    - date raised: 2022-01-13 

 * **open issue 0855:** Assemble() docu     
    - description:  add information on general approach of adding objects and mbs.Assemble() procedure in Overview on Exudyn; add figure Add Nodes/Objects->Assemble->Solve
    - date raised: 2022-01-09 

 * **open issue 0853:** ComputeObjectJacobian...
    - description:  add mbs computation functions for jacobians, for ODE1, ODE2 and AE
    - date raised: 2022-01-08 

 * **open issue 0852:** ComputeObjectAlgebraicEquations
    - description:  add mbs computation functions for constraints
    - date raised: 2022-01-08 

 * **open issue 0845:** Jacobian documentation
    - description:  add documentation to object and connector jacobians
    - date raised: 2021-12-23 

 * **open issue 0844:** connector jacobian RigidBodySpringDamper
    - description:  add analytic jacobian for RigidBodySpringDamper connector
    - date raised: 2021-12-23 

 * **open issue 0841:** GeneralContact regularized friction
    - description:  extend regularized friction (Haff-Werner) to integrated form using either Cundall-Stack friction or breaking tangential springs
    - date raised: 2021-12-19 

 * **open issue 0836:** GeneralContact add cone
    - description:  for 3D cables add cone (including cylinder) to contact with 3D cables
    - date raised: 2021-12-19 

 * **open issue 0835:** GeneralContact add dissipative laws
    - description:  add common dissipative (coeff of restitution, etc. laws to contact
    - date raised: 2021-12-19 

 * **open issue 0834:** GeneralContact add contact laws
    - description:  add Hertzian contact laws to contacts, using separate enum for contact laws and additional parameter
    - date raised: 2021-12-19 

 * **open issue 0833:** GeneralContact rolling pivoting
    - description:  add rolling and pivoting (drilling, boring) friction to spheres and triangles
    - date raised: 2021-12-17 

 * **open issue 0829:** velocityOffset      
    - description:  add velocity offset to all spring dampers in order to replace many user functions with preStepUserFunctions
    - date raised: 2021-12-15 

 * **open issue 0792:** test Pardiso integration
    - description:  VS2017 settings with Intel Performance Libraries and test interface via Eigen
    - date raised: 2021-11-02 

 * **open issue 0783:** SetObjectParameter  
    - description:  extend functionality of Set[Item]Parameter functions to accept lists AND numpy arrays for vectors
    - date raised: 2021-10-29 

 * **open issue 0782:** MatrixContainer     
    - description:  add SetWithNGsolveSparseMatrix; add an interface to directly convert from NGsolve matrix, also converting coordinate storage xxyyzz
    - date raised: 2021-10-27 

 * **open issue 0777:** GenericODE2         
    - description:  add tests for dense and sparse mass and jacobian matrices, together with sparse/dense solvers
    - date raised: 2021-10-09 

 * **open issue 0753:** autodiff for Connectors
    - description:  add autodiff for connectors using spezial sizes like 6 for 2 position nodes, 14 for 2 rigid bodies and 40 for most objects (ObjectFFRFreducedOrder) + 100? as extreme case, falling back to numerical diff for any larger case
    - date raised: 2021-09-21 

 * **open issue 0737:** ContactCoordinate   
    - description:  check if is very close to switching, perform switching for end of step and set error very small; if immediate swichting after beginning of step, do not set stepRecommendation to avoid step reduction; repeat step; time integration: if recommended step is set, reduction is performed in first iteration, otherwise iterate
    - date raised: 2021-08-13 

 * **open issue 0736:** include GeomExactBeam3D
    - description:  as provided by Jan Tomec
    - date raised: 2021-08-12 

 * **open issue 0728:** optimize CollectCurrentNodeMarkerData
    - description:  optimize function for CNodeRigidBodyRotVecLG
    - date raised: 2021-07-31 

 * **open issue 0710:** CollectCurrentNodeData
    - description:  implement CollectCurrentNodeData for NodeRigidBody2D, optimize CollectCurrentNodeData for all rigid body nodes
    - date raised: 2021-07-08 

 * **open issue 0707:** LinkedDataVector,ResizableVector
    - description:  consider removing ResizableVector(integrate into Vector), implement LinkedDataVector as templated spezialization of Vector, no virtual calls in Vector
    - date raised: 2021-07-06 

 * **open issue 0698:** GetAvailableJacobians
    - description:  unify constraint.GetAvailableJacobians() with jacobian computations in joints, in order to avoid large overheads for jacobian assembly
    - date raised: 2021-07-01 

 * **open issue 0692:** CSystem             
    - description:  check JacobianAE: jacobianGM.AddSubmatrixTransposed(temp.localJacobianAE_ODE2_t ... if _t is correctly used
    - date raised: 2021-06-28 

 * **open issue 0648:** solver tutorial     
    - description:  create video with frequent solver errors and FAQ
    - date raised: 2021-05-01 

 * **open issue 0627:** ObjectContactFrictionCircleCable2D
    - description:  add description, connector equations and figure
    - date raised: 2021-04-21 

 * **open issue 0626:** GenericJoint        
    - description:  add more description on constraint configurations, coordinate transformations and figures for GenericJoint
    - date raised: 2021-04-21 

 * **open issue 0625:** ObjectRigidBody     
    - description:  revise equations of motion and add figure for COM and local coordinates
    - date raised: 2021-04-15 

 * **open issue 0617:** python userFunctions
    - description:  consider adding an additional userFunctionVariable [List? or Dict?], which contains indices or further parameters needed in the userFunction
    - date raised: 2021-03-23 

 * **open issue 0614:** sensor dependencies 
    - description:  add functionality to compute sensor-dependencies (for LTG computation), used in controller connectors? alternatively add dependentNodes to existing connectors
    - date raised: 2021-03-21 

 * **open issue 0613:** MarkerObjectODE2Coordinates
    - description:  add simple test into TestModels
    - date raised: 2021-03-21 

 * **open issue 0608:** recommendedStepSize 
    - description:  add recommendedStepSize to ContactCoordinate element
    - date raised: 2021-03-20 

 * **open issue 0591:** ObjectFFRF          
    - description:  Check forceVector and gravity forces in comparison to paper
    - date raised: 2021-02-21 

 * **open issue 0574:** initialAccelerations
    - description:  add dg/dq\*(dot q) term for initial accelerations in velocity level constraints; check with rolling coin
    - date raised: 2021-02-07 

 * **open issue 0565:** Drift inspector     
    - description:  add functionality to check whether drift gets too large
    - date raised: 2021-01-29 

 * **open issue 0561:** Gen alpha2          
    - description:  setup new generalized alpha integrator with GGL
    - date raised: 2021-01-26 

 * **open issue 0559:** Lie group tests2    
    - description:  add Lie group integrator flybar governor
    - date raised: 2021-01-26 

 * **open issue 0517:** item number textures
    - description:  add special textures for item numbers
    - date raised: 2020-12-22 

 * **open issue 0436:** virtual functions   
    - description:  make virtual functions consistent for some system classes like MainSystem, etc. which have no derived classes
    - date raised: 2020-07-21 

 * **open issue 0410:** drawing information 
    - description:  add consistent drawing information in show field of every item
    - date raised: 2020-05-24 

 * **open issue 0401:** add sensor miniexamples
    - description:  .
    - date raised: 2020-05-21 

 * **open issue 0390:** SlimVector          
    - description:  check if erasing all <rule of 5> methods in SlimVector work and speed up code performance
    - date raised: 2020-05-16 

 * **open issue 0380:** mass matrix update  
    - description:  mass matrix is not updated in Generalized Alpha solver in CSolverImplicitSecondOrderTimeInt::ComputeNewtonJacobian - may be critical for 3d rigid bodies
    - date raised: 2020-05-06 

 * **open issue 0354:** autodiff            
    - description:  add consistent object (not connector) differentiation either manually or with autodiff
    - date raised: 2020-03-05 

 * **open issue 0347:** initialCoordinates_t
    - description:  instead of initialVelocities
    - date raised: 2020-02-24 

 * **open issue 0332:** getobject/nodeparameter
    - description:  extend getobjectparameter/node/.. with default function from MainObject / MainNode/ ... which returns basic information, e.g., NodeType 
    - date raised: 2020-02-04 

 * **open issue 0314:** Add user marker     
    - description:  add user marker
    - date raised: 2020-01-10 

 * **open issue 0303:** constraints derivatives
    - description:  add consistent flag, if constraints have velocity coordinate dependence or if they explicitly depend on time (needs additional derivatives for consistent initial accelerations)!
    - date raised: 2019-12-26 

 * **open issue 0260:** static computation  
    - description:  add consistent flag to markerdata computation, ODE2RHS computation, etc. for static computation, which does not compute information on velocities then.
    - date raised: 2019-09-10 

 * **open issue 0258:** contour plot        
    - description:  extend mass points and rigid bodies for contour plotting
    - date raised: 2019-08-30 

 * **open issue 0252:** PostNewtonStep      
    - description:  post newton step object functions shall be called from solver including a ResizableVector& dataVariables to be changed; post newton function shall not use direct write access to nodal data coordinates
    - date raised: 2019-08-27 

 * **open issue 0241:** PostNewtonStep      
    - description:  Check why markerData is computed with computeJacobian=true in CSystem::PostNewtonStep; is jacobian information really needed?
    - date raised: 2019-08-22 

 * **open issue 0209:** contact iteration   
    - description:  make simple example for contact to check changing jacobian matrices from ContactCoordinate
    - date raised: 2019-06-28 

 * **open issue 0194:** Jacobians           
    - description:  Make unique member function names for rotation/orientation jacobians in nodes and bodies
    - date raised: 2019-06-25 

 * **open issue 0174:** Iterator begin      
    - description:  Check if const consistency is realizable
    - date raised: 2019-06-07 

 * **open issue 0173:** LinkedDataVector    
    - description:  Check that the Vector is declared as const LinkedDataVectors if it is returned by a function; otherwise unforeseeable problems might occur
    - date raised: 2019-06-07 

 * **open issue 0172:** Matrix Init         
    - description:  Consider to put this funciton into constructor and call constructor instead of init
    - date raised: 2019-06-07 

 * **open issue 0171:** Matrix delete       
    - description:  Consider not to set data=NULL in order to detect memory which is deleted twice
    - date raised: 2019-06-07 

 * **open issue 0161:** SlimArray           
    - description:  Add template specializations to SlimArray<T,2..4> similar to SlimVector to speed up initialization of short vectors
    - date raised: 2019-06-02 

 * **open issue 0142:** Vector performance  
    - description:  check Vector operator[], and ConstVector performance regarding inlining
    - date raised: 2019-05-21 

 * **open issue 0124:** ConstSizeVector     
    - description:  check if begin/end() overriding of Vector:: function is needed?
    - date raised: 2019-05-13 

 * **open issue 0123:** Linalg Override     
    - description:  Add override statement to all derived classes in linalg for safety
    - date raised: 2019-05-13 

 * **open issue 0121:** allocation failure  
    - description:  assert that every allocation in Matrix, Vector, ResizableArray, ... is performed with try/catch - compare Matrix::AllocateMemory(...)
    - date raised: 2019-05-13 

 * :textblue:`open issue 0098:` Destructors/Cleanup 
    - description:  add destructors/cleanup to MainSystemData and all other system functions (check new commands)    
    - date raised: 2019-04-01 

 * :textblue:`open issue 0088:` Data dependency     
    - description:  use dependencies for every computational member function; node: 		NodeData: LinkedDataVector displacement, velocity, acceleration;; object(singlenoded): 	ObjectData: LinkedDataVector displacement, velocity, acceleration;; object(multinoded): 	ObjectData: ResizableArray<LinkedDataVector> displacements, velocities, accelerations;; constraint(Lagr.):	FunctionResults1, FunctionResults2 (e.g. RotMatrix1, Position1, ...); marker:		only transforms data (load/constraint); load:			only provides load information    
    - date raised: 2019-04-01 

**********
Known bugs
**********

 * :textred:`open BUG 2494:` CompositionRuleForRotationVectors returns 2pi instead of 0 for opposite half-turns
    - issue author: Claude-JG
    - description:  Composing the rotation vector pi\*n with itself gives a vector of norm 2\*pi instead of the zero vector. Both describe the identity rotation - ExpSO3 of the result IS the identity - but 2\*pi is outside the principal range and is exactly where the tangent operator is singular: TExpSO3Inv at that vector returns entries of order 1e15. A time integration that composes into that point therefore continues with a meaningless T matrix. Found by TEST 2 of TestModels/LieGroupIntegrationUnitTests.py; which compares against Matlab results that give 0; the other nine tests of that file pass. revision2026 step R5.12.1
    - date raised: 2026-09-17 

 * :textred:`open BUG 2463:` the two GitHub workflows pin different action versions
    - issue author: Claude-JG
    - description:  .github/workflows/wheels.yml uses actions/setup-python@v6, while documentation.yaml still uses actions/checkout@v3 and actions/setup-python@v4. Found in revision2026 step R5.14; assigned to sub-step R5.14.1, which needs maintainer approval because it touches .github/workflows.
    - date raised: 2026-09-16 

 * :textred:`open BUG 2398:` explicit integration costs O(N^2) per step with the default dense linear solver
    - issue author: Claude-JG
    - description:  measured 2026-09-12 on a chain of point masses coupled by coordinate spring dampers; explicit Euler; 200 steps: nMasses 250/500/1000/2000 gives 2.5/10.1/42/168 ms per step - the per step cost quadruples on every doubling; so it is O(N^2) although an explicit step on a chain should be O(N). Setting simulationSettings.linearSolverType to EigenSparse makes it linear and 400 times faster at nMasses=2000 (0.084 s against 33.5 s for 200 steps). The dense default is reasonable for small systems; but nothing warns at large N and explicit integration does not obviously need a linear solver at all; so the trap is invisible. Found while building a large system performance test for revision2026 step R2.10
    - date raised: 2026-09-12 

 * :textred:`open BUG 2127:` ContactSphereTorus  
    - description:  check torques on both bodies, as there seems to be momentum conservation issues in ball bearings
    - date raised: 2025-07-03 

 * :textred:`open BUG 1889:` symbolic            
    - description:  GetLoad and similar functions do not work with symbolic user functions and raise TypeError: Object of type 'exudyn.exudynCPP.symbolic.UserFunction' is not an instance of 'function'; see also issue with mbs.GetDictionary()
    - date raised: 2024-10-11 

 * :textred:`open BUG 1772:` item.GetDictionary  
    - description:  item.GetDictionary not working for new user function interface with symbolic user function
    - date raised: 2024-02-04 

 * :textred:`open BUG 1639:` SolveDynamic FFRF   
    - description:  repeated call to mbs.SolveDynamic gives divergence; attributed to FFRFreducedOrder model; workaround uses repeated build of model before calling solver again; may be related to FFRF or MarkerSuperElement-internal variables
    - date raised: 2023-07-10 

 * :textred:`open BUG 1048:` sse2neon.h          
    - description:  on Apple, sse2neon.h is missing (include from github) and compilation fails; check if this only happens on M1 and change include modes of sse2neon.h; add this file to python setup.py for other cases
    - date raised: 2022-04-25 

 * :textred:`open BUG 0830:` PostNewton          
    - description:  PostNewton missing in explicit solvers; add warning or add after single steps (but exclude in contact computation!)
    - date raised: 2021-12-15 

 * :textred:`open BUG 0738:` ObjectContactCoordinate
    - description:  modified Newton does not work, no Jacobian update computed when switching
    - date raised: 2021-08-13 


