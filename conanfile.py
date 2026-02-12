from conan import ConanFile

class MainProject(ConanFile):
    python_requires = "conan_template/[~5]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_pdpreprocessor"
    description = "module_pdpreprocessor is used to generate deterministic triggers for other modules."
    exports_sources = ["*", "!.gitignore"]

    def requirements(self):
        self.requires("robotkernel/6.0.0-yaml-service@robotkernel/unstable")
        self.requires("service_provider_process_data_inspection/6.0.0-yaml-service@robotkernel/unstable")
        self.requires("service_provider_key_value/6.0.0-yaml-service@robotkernel/unstable")

    def source(self):
        self.run(f"sed 's/AC_INIT(.*/AC_INIT([{self.name}], [{self.version}], [{self.author}])/' configure.ac.in > configure.ac")

