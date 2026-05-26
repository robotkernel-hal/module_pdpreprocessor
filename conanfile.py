from conan import ConanFile

class MainProject(ConanFile):
    python_requires = "conan_template/[~6]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_pdpreprocessor"
    description = "module_pdpreprocessor is used to generate deterministic triggers for other modules."
    exports_sources = ["*", "!.gitignore"]

    def requirements(self):
        self.requires("robotkernel/[~6]@robotkernel/stable")
        self.requires("service_provider_process_data_inspection/[~6]@robotkernel/stable")
        self.requires("service_provider_key_value/[~6]@robotkernel/stable")

