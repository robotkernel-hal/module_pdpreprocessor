from conans import tools, python_requires

base = python_requires("conan_template/[~=5]@robotkernel/stable")

class MainProject(base.RobotkernelConanFile):
    name = "module_pd_preprocessor"
    description = "module_pd_preprocessor is used to generate deterministic triggers for other modules."
    exports_sources = ["*", "!.gitignore"] + ["!%s" % x for x in tools.Git().excluded_files()]

    def requirements(self):
        self.requires("robotkernel/[>=5.0.19]@robotkernel/stable")
        self.requires("service_provider_process_data_inspection/[~=5]@robotkernel/stable")

