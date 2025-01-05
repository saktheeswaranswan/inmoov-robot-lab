angular.module('mrlapp.service.EmailGui', []).controller('EmailGuiCtrl', ['$scope', 'mrl', function($scope, mrl) {
    console.info('EmailGuiCtrl')
    var _self = this
    var msg = this.msg

    // GOOD TEMPLATE TO FOLLOW
    this.updateState = function(service) {
        $scope.service = service
    }

    msg.subscribe(this)
}
])
